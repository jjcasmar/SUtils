#ifndef WIREFRAMEMATERIAL_H
#define WIREFRAMEMATERIAL_H

#include <Qt3DRender>

namespace Qt3DRender {
class QFilterKey;
class QEffect;
class QTechnique;
class QParameter;
class QShaderProgram;
class QRenderPass;
class QCullFace;
}

class WireframeMaterial : public Qt3DRender::QMaterial
{
public:
    WireframeMaterial(Qt3DCore::QNode *parent = nullptr);
    void init();

    qreal m_lineWidth;
    QColor m_lineColor;

    Qt3DRender::QEffect *m_vertexEffect;
    Qt3DRender::QTechnique *m_vertexGL3Technique;
    Qt3DRender::QRenderPass *m_vertexGL3RenderPass;
    Qt3DRender::QShaderProgram *m_vertexGL3Shader;
    Qt3DRender::QParameter *m_lineWidthParameter;
    Qt3DRender::QParameter *m_lineColorParameter;
    Qt3DRender::QCullFace *m_cullFaceState;
    Qt3DRender::QFilterKey *m_filterKey;
};

#endif // WIREFRAMEMATERIAL_H
