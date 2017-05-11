#include "wireframematerial.h"

#include <Qt3DRender>

WireframeMaterial::WireframeMaterial(Qt3DCore::QNode *parent) :
    Qt3DRender::QMaterial(parent)
  , m_lineWidth(0.8)
  , m_lineColor(QColor(0,0,0))
  , m_vertexEffect(new Qt3DRender::QEffect())
  , m_vertexGL3Technique(new Qt3DRender::QTechnique)
  , m_vertexGL3RenderPass(new Qt3DRender::QRenderPass)
  , m_vertexGL3Shader(new Qt3DRender::QShaderProgram)
  , m_lineWidthParameter(new Qt3DRender::QParameter(QStringLiteral("line.width"), 0.8))
  , m_lineColorParameter(new Qt3DRender::QParameter(QStringLiteral("line.color"), QColor(0,0,0)))
  , m_filterKey(new Qt3DRender::QFilterKey)
{
    init();
}

void WireframeMaterial::init()
{
    m_vertexGL3Shader->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl::fromLocalFile("/home/jjcasmar/Doctorado/SUtils/src/shaders/robustwireframe.vert")));
    m_vertexGL3Shader->setGeometryShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl::fromLocalFile("/home/jjcasmar/Doctorado/SUtils/src/shaders/robustwireframe.geom")));
    m_vertexGL3Shader->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl::fromLocalFile("/home/jjcasmar/Doctorado/SUtils/src/shaders/robustwireframe.frag")));

    m_vertexGL3Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
    m_vertexGL3Technique->graphicsApiFilter()->setMajorVersion(3);
    m_vertexGL3Technique->graphicsApiFilter()->setMinorVersion(1);
    m_vertexGL3Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);

    m_filterKey->setParent(this);
    m_filterKey->setName(QStringLiteral("renderingStyle"));
    m_filterKey->setValue(QStringLiteral("forward"));

    m_vertexGL3Technique->addFilterKey(m_filterKey);

    m_vertexGL3RenderPass->setShaderProgram(m_vertexGL3Shader);

    m_vertexGL3Technique->addRenderPass(m_vertexGL3RenderPass);

    m_vertexEffect->addTechnique(m_vertexGL3Technique);
    m_vertexEffect->addParameter(m_lineWidthParameter);
    m_vertexEffect->addParameter(m_lineColorParameter);
    this->setEffect(m_vertexEffect);
}
