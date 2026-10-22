#include "scenerenderer.h"
#include <QPainter>

QImage SceneRenderer::robotImage(int index, const SimulationSettings& settings)
{
    const bool blue = index < settings.Robots_Count();
    const int id = settings.Robots_Count() ? index % settings.Robots_Count() : 0;
    QImage image(QString(":/%1%2.png").arg(blue ? 'b' : 'y').arg(id));
    const QColor color(QString::fromStdString(blue ? settings.ColorRobotBlue() : settings.ColorRobotYellow()));
    if (color == QColor(63, 63, 63)) return image;
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x)
            if (image.pixelColor(x, y) == QColor(63, 63, 63)) image.setPixelColor(x, y, color);
    return image;
}

void SceneRenderer::initialize(const SceneSnapshot& scene)
{
    if (generation == scene.generation) return;
    graphics.clearTextures();
    QImage ground(":/grass.png");
    graphics.loadTexture(&ground);
    const int count = scene.settings.Robots_Count();
    for (int i = 0; i < count * 2; ++i) {
        QImage image = robotImage(i, scene.settings);
        graphics.loadTexture(&image);
    }
    for (int team = 0; team < 2; ++team) {
        for (int i = 0; i < count; ++i) {
            QImage image(32, 32, QImage::Format_ARGB32);
            image.fill(Qt::transparent);
            QPainter painter(&image);
            painter.setPen(team ? QColor(255, 255, 0) : QColor(15, 193, 225));
            QFont font; font.setBold(true); font.setPointSize(26);
            painter.setFont(font);
            painter.drawText(QRect(1, 1, 30, 30), Qt::AlignCenter, QString::number(i));
            painter.end();
            graphics.loadTexture(&image);
        }
    }
    for (int i = 0; i < 6; ++i) {
        const char axis = i % 3 == 0 ? 'x' : i % 3 == 1 ? 'y' : 'z';
        QImage negative(QString(":/sky/neg_%1.png").arg(axis));
        QImage positive(QString(":/sky/pos_%1.png").arg(axis));
        graphics.loadTexture(&negative);
        graphics.loadTexture(&positive);
    }
    QImage wheel(":/wheel.png");
    graphics.loadTexture(&wheel);
    generation = scene.generation;
}

void SceneRenderer::draw(const SceneSnapshot& scene, int width, int height,
                         const RenderQuality& quality, int highlighted,
                         bool showCursor, const std::array<double, 3>& cursor, double cursorRadius)
{
    initialize(scene);
    graphics.setLowLod(quality.lowSpec);
    graphics.initScene(width, height, 0, 0.7, 1);
    const auto& settings = scene.settings;
    for (const auto& primitive : scene.primitives) {
        if (!primitive.visible) continue;
        auto color = primitive.color;
        if (primitive.robotChassis && primitive.robotIndex == highlighted) {
            color[0] *= 2; color[1] *= 1.5; color[2] *= 1.5;
        }
        graphics.setColor(color[0], color[1], color[2], 1);
        const auto* p = primitive.position.data();
        const auto* r = primitive.rotation.data();
        const auto* d = primitive.dimensions.data();
        switch (primitive.kind) {
        case PrimitiveKind::Sphere: graphics.drawSphere(p, r, d[0]); break;
        case PrimitiveKind::Box: graphics.drawBox(p, r, d); break;
        case PrimitiveKind::Cylinder:
            if (primitive.texture < 0) graphics.drawCylinder(p, r, d[1], d[0]);
            else graphics.drawCylinder_TopTextured(p, r, d[1], d[0], primitive.texture, primitive.robotChassis);
            break;
        case PrimitiveKind::Ground:
            graphics.useTexture(primitive.texture);
            graphics.drawGround();
            graphics.noTexture();
            graphics.drawSSLGround(settings.Field_Rad(), settings.Field_Length(), settings.Field_Width(),
                                  settings.Field_Penalty_Depth(), settings.Field_Penalty_Width(),
                                  settings.Field_Penalty_Point(), settings.Field_Line_Width(), 0.0001);
            break;
        case PrimitiveKind::None: break;
        }
    }
    if (quality.showSkybox) {
        const int first = 4 * settings.Robots_Count() + 7;
        graphics.drawSkybox(first, first + 1, first + 2, first + 3, first + 4, first + 5);
    }
    if (showCursor) {
        graphics.setColor(1, 0.9, 0.2, 0.5);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        graphics.drawCircle(cursor[0], cursor[1], 0.001, cursorRadius);
        glDisable(GL_BLEND);
    }
    graphics.finalizeScene();
}
