#include "cadmaterial.h"

#include <Qt3DRender/qfilterkey.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qshaderprogram.h>
#include <Qt3DRender/qparameter.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/QDepthTest>
#include <Qt3DRender/qgraphicsapifilter.h>
#include <QtCore/QUrl>
#include <QColor>
#include <QtGui/QVector3D>
#include <QtGui/QVector4D>

using namespace Qt3DRender;

CADMaterial::CADMaterial(Qt3DCore::QNode *parent)
    : Qt3DRender::QMaterial (parent)
    , m_cadEffect(new QEffect())
    , m_ambientParameter(new QParameter(QStringLiteral("ka"), QColor::fromRgbF(0.05f, 0.05f, 0.05f, 1.0f)))
    , m_diffuseParameter(new QParameter(QStringLiteral("kd"), QColor::fromRgbF(0.7f, 0.7f, 0.7f, 1.0f)))
    , m_specularParameter(new QParameter(QStringLiteral("ks"), QColor::fromRgbF(0.01f, 0.01f, 0.01f, 1.0f)))
    , m_shininessParameter(new QParameter(QStringLiteral("shininess"), 150.0f))
    , m_edgeTypeParameter(new QParameter(QStringLiteral("edgeTypeIsHard"), 1))
    , m_hardEdgeLimitParameter(new QParameter(QStringLiteral("hardEdgeLimite"), 60.0f))
    , m_lineWidthParameter(new Qt3DRender::QParameter(QStringLiteral("line.width"), 1.5))
    , m_lineColorParameter(new Qt3DRender::QParameter(QStringLiteral("line.color"), QColor(0,0,0)))
    , m_cadGL3Technique(new QTechnique())
    , m_phongGL3RenderPass(new QRenderPass())
    , m_wireframeGL3RenderPass(new QRenderPass())
    , m_wireframeDepthTest(new QDepthTest())
    , m_phongGL3Shader(new QShaderProgram())
    , m_wireframeGL3Shader(new QShaderProgram())
    , m_filterKey(new QFilterKey())
    , m_materialEffects(Wireframe | Phong)
{
    m_phongGL3Shader->setVertexShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/phong.vert"))));
//    m_phongGL3Shader->setGeometryShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/phong.geom"))));
    m_phongGL3Shader->setFragmentShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/phong.frag"))));

    m_wireframeGL3Shader->setVertexShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/robustwireframe.vert"))));
    m_wireframeGL3Shader->setGeometryShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/robustwireframe.geom"))));
    m_wireframeGL3Shader->setFragmentShaderCode(QShaderProgram::loadSource(QUrl(QStringLiteral("qrc:/shaders/robustwireframe.frag"))));

    m_cadGL3Technique->graphicsApiFilter()->setApi(QGraphicsApiFilter::OpenGL);
    m_cadGL3Technique->graphicsApiFilter()->setMajorVersion(3);
    m_cadGL3Technique->graphicsApiFilter()->setMinorVersion(3);
    m_cadGL3Technique->graphicsApiFilter()->setProfile(QGraphicsApiFilter::CoreProfile);

    m_phongGL3RenderPass->setShaderProgram(m_phongGL3Shader);
    m_wireframeGL3RenderPass->setShaderProgram(m_wireframeGL3Shader);
    m_wireframeDepthTest->setDepthFunction(QDepthTest::LessOrEqual);
    m_wireframeGL3RenderPass->addRenderState(m_wireframeDepthTest);

    m_filterKey->setParent(this);
    m_filterKey->setName(QStringLiteral("renderingStyle"));
    m_filterKey->setValue(QStringLiteral("forward"));
    m_cadGL3Technique->addFilterKey(m_filterKey);

    m_cadGL3Technique->addRenderPass(m_phongGL3RenderPass);
    m_cadGL3Technique->addRenderPass(m_wireframeGL3RenderPass);

    m_cadEffect->addTechnique(m_cadGL3Technique);

    m_phongGL3RenderPass->addParameter(m_ambientParameter);
    m_phongGL3RenderPass->addParameter(m_diffuseParameter);
    m_phongGL3RenderPass->addParameter(m_specularParameter);
    m_phongGL3RenderPass->addParameter(m_shininessParameter);
    m_phongGL3RenderPass->addParameter(m_edgeTypeParameter);
    m_phongGL3RenderPass->addParameter(m_hardEdgeLimitParameter);
    m_wireframeGL3RenderPass->addParameter(m_lineWidthParameter);
    m_wireframeGL3RenderPass->addParameter(m_lineColorParameter);

    setEffect(m_cadEffect);
}

CADMaterial::~CADMaterial()
{

}

QColor CADMaterial::ambient() const
{
    return m_ambientParameter->value().value<QColor>();
}

QColor CADMaterial::diffuse() const
{
    return m_diffuseParameter->value().value<QColor>();
}

QColor CADMaterial::specular() const
{
    return m_specularParameter->value().value<QColor>();
}

float CADMaterial::shininess() const
{
    return m_shininessParameter->value().toFloat();
}

CADMaterial::EdgeType CADMaterial::edgeType() const
{
    return m_edgeTypeParameter->value().value<EdgeType>();
}

float CADMaterial::hardEdgeLimit() const
{
    return m_hardEdgeLimitParameter->value().toFloat();
}

CADMaterial::Effects CADMaterial::materialEffects() const
{
    return m_materialEffects;
}

void CADMaterial::setAmbient(const QColor &ambient)
{
    m_ambientParameter->setValue(ambient);
    emit ambientChanged(ambient);
}

void CADMaterial::setDiffuse(const QColor &diffuse)
{
    m_diffuseParameter->setValue(diffuse);
    emit diffuseChanged(diffuse);
}

void CADMaterial::setSpecular(const QColor &specular)
{
    m_specularParameter->setValue(specular);
    emit specularChanged(specular);
}

void CADMaterial::setShininess(float shininess)
{
    m_shininessParameter->setValue(shininess);
    emit shininessChanged(shininess);
}

void CADMaterial::setEdgeType(CADMaterial::EdgeType edgeType)
{
    m_edgeTypeParameter->setValue(QVariant::fromValue(edgeType));
    emit edgeTypeChanged(edgeType);
}

void CADMaterial::setHardEdgeLimit(float hardEdgeLimit)
{
    m_hardEdgeLimitParameter->setValue(hardEdgeLimit);
    emit hardEdgeLimitChanged(hardEdgeLimit);
}

void CADMaterial::setMaterialEffects(const Effects &materialEffects)
{
    if (m_materialEffects != materialEffects) {
        m_materialEffects = materialEffects;

        m_cadGL3Technique->removeRenderPass(m_phongGL3RenderPass);
        m_cadGL3Technique->removeRenderPass(m_wireframeGL3RenderPass);

        if (materialEffects & Phong)
            m_cadGL3Technique->addRenderPass(m_phongGL3RenderPass);

        if (materialEffects & Wireframe)
            m_cadGL3Technique->addRenderPass(m_wireframeGL3RenderPass);

        emit materialEffectsChanged(materialEffects);
    }
}


