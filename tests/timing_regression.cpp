#include <QApplication>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QThread>
#include <QTimer>
#include <QUdpSocket>
#include <cmath>
#include <cstdio>
#include "mainwindow.h"
#include "ssl_vision_wrapper.pb.h"
#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"

// This process has its own event loop: blocking the GUI must not also block
// the observer and accidentally hide a pause in network publication.
static int observe(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const quint16 controlPort = QString::fromLocal8Bit(argv[2]).toUShort();
    QUdpSocket vision, feedback;
    if (!vision.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !feedback.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    std::printf("READY %u\n", vision.localPort());
    std::fflush(stdout);
    QElapsedTimer clock;
    clock.start();
    qint64 firstArrival = -1, lastArrival = 0, longestGap = 0;
    qint64 sent = -1, feedbackLatency = 0;
    int frames = 0, responses = 0;
    quint32 previousFrame = 0;
    double firstTime = 0, lastTime = 0;
    bool fixedSteps = true;
    QObject::connect(&vision, &QUdpSocket::readyRead, [&]() {
        while (vision.hasPendingDatagrams()) {
            QByteArray data(vision.pendingDatagramSize(), 0);
            vision.readDatagram(data.data(), data.size());
            SSL_WrapperPacket packet;
            if (!packet.ParseFromArray(data.data(), data.size()) || !packet.has_detection() ||
                packet.detection().camera_id() != 0) continue;
            const auto& detection = packet.detection();
            const qint64 now = clock.nsecsElapsed();
            if (frames) {
                longestGap = qMax(longestGap, now - lastArrival);
                fixedSteps = fixedSteps && detection.frame_number() == previousFrame + 1 &&
                    std::abs(detection.t_capture() - lastTime - 1.0 / 60.0) < 1e-9;
            } else {
                firstArrival = now;
                firstTime = detection.t_capture();
            }
            previousFrame = detection.frame_number();
            lastArrival = now;
            lastTime = detection.t_capture();
            ++frames;
        }
    });
    QObject::connect(&feedback, &QUdpSocket::readyRead, [&]() {
        while (feedback.hasPendingDatagrams()) {
            QByteArray data(feedback.pendingDatagramSize(), 0);
            feedback.readDatagram(data.data(), data.size());
            RobotControlResponse response;
            if (!response.ParseFromArray(data.data(), data.size()) || response.feedback_size() == 0) continue;
            if (sent >= 0) feedbackLatency = qMax(feedbackLatency, clock.nsecsElapsed() - sent);
            sent = -1;
            ++responses;
        }
    });
    QTimer commands;
    commands.setTimerType(Qt::PreciseTimer);
    commands.setInterval(20);
    QObject::connect(&commands, &QTimer::timeout, [&]() {
        if (sent >= 0) return;
        RobotControl command;
        auto* robot = command.add_robot_commands();
        robot->set_id(0);
        auto* velocity = robot->mutable_move_command()->mutable_local_velocity();
        velocity->set_forward(0); velocity->set_left(0); velocity->set_angular(0);
        const std::string data = command.SerializeAsString();
        sent = clock.nsecsElapsed();
        feedback.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, controlPort);
    });
    commands.start();
    QTimer::singleShot(1400, &app, &QCoreApplication::quit);
    app.exec();
    const double elapsed = firstArrival >= 0 ? (lastArrival-firstArrival)/1e9 : 0;
    const double simulated = lastTime-firstTime;
    const bool passed = frames > 50 && responses > 20 && fixedSteps &&
        longestGap < 75000000 && feedbackLatency < 75000000 && std::abs(simulated-elapsed) < 0.08;
    QJsonObject result;
    result["passed"] = passed;
    result["frames"] = frames;
    result["responses"] = responses;
    result["max_gap_ms"] = longestGap/1e6;
    result["max_feedback_ms"] = feedbackLatency/1e6;
    result["simulation_s"] = simulated;
    result["wall_s"] = elapsed;
    const QByteArray json = QJsonDocument(result).toJson(QJsonDocument::Compact);
    std::printf("%s\n", json.constData());
    return passed ? 0 : 1;
}

static bool scenario(MainWindow& window, GLWidget& view, quint16 controlPort, double fps, int stall)
{
    QProcess observer;
    observer.start(QCoreApplication::applicationFilePath(), {"--observe", QString::number(controlPort)});
    if (!observer.waitForStarted(5000)) return false;
    QElapsedTimer timeout;
    timeout.start();
    while (!observer.canReadLine() && timeout.elapsed() < 5000) observer.waitForReadyRead(100);
    const QByteArray ready = observer.readLine().trimmed();
    if (!ready.startsWith("READY ")) { observer.kill(); observer.waitForFinished(); return false; }
    view.cfg->v_VisionMulticastPort->setInt(ready.mid(6).toUShort());
    view.cfg->v_DesiredFPS->setDouble(fps);
    window.reconnectVisionSocket();
    window.changeTimer();
    QEventLoop loop;
    QObject::connect(&observer, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), &loop, &QEventLoop::quit);
    if (stall) QTimer::singleShot(200, [&]() { QThread::msleep(stall); });
    QTimer::singleShot(10000, &loop, &QEventLoop::quit);
    loop.exec();
    if (observer.state() != QProcess::NotRunning) { observer.kill(); observer.waitForFinished(); return false; }
    const QByteArray output = observer.readAllStandardOutput().trimmed();
    std::printf("%s fps=%.0f gui_stall=%dms %s\n", observer.exitCode() == 0 ? "PASS" : "FAIL", fps, stall, output.constData());
    if (observer.exitCode()) std::fprintf(stderr, "%s", observer.readAllStandardError().constData());
    return observer.exitCode() == 0;
}

int main(int argc, char** argv)
{
    if (argc > 2 && QByteArray(argv[1]) == "--observe") return observe(argc, argv);
    QApplication app(argc, argv);
    QUdpSocket reserve;
    if (!reserve.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    const quint16 controlPort = reserve.localPort();
    reserve.close();
    auto* config = new ConfigWidget(false);
    config->v_DeltaTime->setDouble(1.0/60.0);
    config->v_DesiredFPS->setDouble(60);
    config->v_Robots_Count->setInt(1);
    config->v_sendDelay->setInt(0);
    config->v_sendGeometryEvery->setInt(120);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(0);
    config->v_BlueControlListenPort->setInt(controlPort);
    config->v_YellowControlListenPort->setInt(0);
    auto* window = new MainWindow(nullptr, config);
    window->hide();
    window->setIsGlEnabled(false);
    auto* view = window->findChild<GLWidget*>();
    if (!view) return 2;
    bool passed = scenario(*window, *view, controlPort, 60, 0);
    passed = scenario(*window, *view, controlPort, 5, 0) && passed;
    passed = scenario(*window, *view, controlPort, 1, 0) && passed;
    passed = scenario(*window, *view, controlPort, 60, 500) && passed;
    delete window;
    return passed ? 0 : 1;
}
