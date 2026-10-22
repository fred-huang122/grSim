#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QThread>
#include <QUdpSocket>
#include <cmath>
#include <cstdio>
#include <functional>
#include "mainwindow.h"
#include "ssl_simulation_control.pb.h"
#include "ssl_gc_common.pb.h"

static bool waitFor(const std::function<bool()>& predicate, bool deliverGuiEvents = false)
{
    QElapsedTimer clock;
    clock.start();
    while (clock.elapsed() < 2000) {
        if (deliverGuiEvents) QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        if (predicate()) return true;
        QThread::msleep(1);
    }
    return predicate();
}

static bool check(bool passed, const char* description)
{
    std::printf("%s %s\n", passed ? "PASS" : "FAIL", description);
    std::fflush(stdout);
    return passed;
}

static bool equal(double a, double b) { return std::abs(a - b) < 1e-6; }

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QUdpSocket reserve, control;
    if (!reserve.bind(QHostAddress(QHostAddress::LocalHost), 0) ||
        !control.bind(QHostAddress(QHostAddress::LocalHost), 0)) return 2;
    const quint16 port = reserve.localPort();
    reserve.close();
    auto* config = new ConfigWidget(false);
    config->v_Robots_Count->setInt(1);
    config->v_CommandListenPort->setInt(0);
    config->v_SimControlListenPort->setInt(port);
    config->v_BlueControlListenPort->setInt(0);
    config->v_YellowControlListenPort->setInt(0);
    config->v_VisionMulticastAddr->setString("127.0.0.1");
    config->v_VisionMulticastPort->setInt(control.localPort());
    MainWindow window(nullptr, config);
    window.hide();
    window.setIsGlEnabled(false);
    auto* simulation = window.findChild<GLWidget*>()->simulation;
    auto sendSpec = [&](Team team, double radius, double mass) {
        SimulatorCommand command;
        auto* spec = command.mutable_config()->add_robot_specs();
        spec->mutable_id()->set_id(0);
        spec->mutable_id()->set_team(team);
        if (radius) spec->set_radius(radius);
        if (mass) spec->set_mass(mass);
        const auto data = command.SerializeAsString();
        return control.writeDatagram(data.data(), int(data.size()), QHostAddress::LocalHost, port)
            == qint64(data.size());
    };
    bool passed = true;

    // Do not deliver GUI events between the remote rebuild and the local edit.
    // This reproduces a busy GUI exporting its older copy of the robot specs.
    const double initialRadius = config->blueSettings.RobotRadius;
    if (!sendSpec(BLUE, 0.10, 3.25) || !waitFor([&]() {
        return equal(simulation->latest()->settings.blueSettings.RobotRadius, 0.10);
    })) return 2;
    passed = check(equal(config->blueSettings.RobotRadius, initialRadius),
                   "remote notification remains queued while the GUI is busy") && passed;
    config->v_noise->setBool(true);
    emit config->settingsEdited();
    if (!waitFor([&]() { return simulation->latest()->settings.noise(); })) return 2;
    passed = check(equal(simulation->latest()->settings.blueSettings.RobotRadius, 0.10) &&
                   equal(simulation->latest()->settings.blueSettings.BodyMass, 3.25),
                   "unrelated GUI edit preserves newer remote robot specifications") && passed;
    passed = check(waitFor([&]() {
        return equal(config->blueSettings.BodyMass, 3.25) &&
            equal(simulation->latest()->settings.blueSettings.BodyMass, 3.25);
    }, true), "GUI receives the merged worker settings") && passed;

    // The GUI may also edit one robot field while another field changes remotely.
    if (!sendSpec(BLUE, 0, 3.5) || !waitFor([&]() {
        return equal(simulation->latest()->settings.blueSettings.BodyMass, 3.5);
    })) return 2;
    config->blueSettings.RobotRadius = 0.11;
    emit config->settingsEdited();
    config->blueSettings.RobotRadius = 0.115;
    config->v_noise->setBool(false);
    emit config->settingsEdited();
    if (!waitFor([&]() {
        return equal(simulation->latest()->settings.blueSettings.RobotRadius, 0.115) &&
            !simulation->latest()->settings.noise();
    })) return 2;
    passed = check(equal(simulation->latest()->settings.blueSettings.BodyMass, 3.5),
                   "GUI robot edit merges with a remote edit to a different field") && passed;
    bool pendingEditPreserved = true;
    const auto connection = QObject::connect(simulation, &SimulationController::remoteSettingsChanged,
        [&](const SimulationSettings&) {
            pendingEditPreserved = pendingEditPreserved && equal(config->blueSettings.RobotRadius, 0.115);
        });
    passed = check(waitFor([&]() {
        return equal(config->blueSettings.RobotRadius, 0.115) && equal(config->blueSettings.BodyMass, 3.5);
    }, true) && pendingEditPreserved,
        "delayed worker notifications preserve newer pending GUI edits") && passed;
    QObject::disconnect(connection);

    if (!sendSpec(YELLOW, 0, 4.0) || !waitFor([&]() {
        return equal(config->yellowSettings.BodyMass, 4.0);
    }, true)) return 2;
    if (!sendSpec(BLUE, 0, 5.0) || !waitFor([&]() {
        return equal(simulation->latest()->settings.blueSettings.BodyMass, 5.0);
    })) return 2;
    // The preset's coalesced GUI notification is queued behind the remote one.
    config->v_BlueTeam->setString("ParsianNew");
    config->v_BlueTeam->wasEdited(config->v_BlueTeam);
    passed = check(waitFor([&]() {
        return simulation->latest()->settings.BlueTeam() == "ParsianNew";
    }, true), "GUI preset change reaches the worker") && passed;
    passed = check(equal(simulation->latest()->settings.blueSettings.BodyMass, 2.0) &&
                   equal(simulation->latest()->settings.yellowSettings.BodyMass, 4.0) &&
                   equal(config->yellowSettings.BodyMass, 4.0),
                   "changing one team preset restores its defaults and preserves the other team") && passed;
    return passed ? 0 : 1;
}
