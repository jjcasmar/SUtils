#ifndef CADMATERIAL_H
#define CADMATERIAL_H

#include <Qt3DRender/QMaterial>
#include <QColor>

namespace Qt3DRender
{
class QFilterKey;
class QEffect;
class QTechnique;
class QParameter;
class QShaderProgram;
class QRenderPass;
class QDepthTest;
}

class CADMaterial : public Qt3DRender::QMaterial
{
    Q_OBJECT
    Q_PROPERTY(QColor ambient READ ambient WRITE setAmbient NOTIFY ambientChanged)
    Q_PROPERTY(QColor diffuse READ diffuse WRITE setDiffuse NOTIFY diffuseChanged)
    Q_PROPERTY(QColor specular READ specular WRITE setSpecular NOTIFY specularChanged)
    Q_PROPERTY(float shininess READ shininess WRITE setShininess NOTIFY shininessChanged)
    Q_PROPERTY(EdgeType edgeType READ edgeType WRITE setEdgeType NOTIFY edgeTypeChanged)
    Q_PROPERTY(float hardEdgeLimit READ hardEdgeLimit WRITE setHardEdgeLimit NOTIFY hardEdgeLimitChanged)
    Q_PROPERTY(Effects materialEffects READ materialEffects WRITE setMaterialEffects NOTIFY materialEffectsChanged)

    Q_FLAGS(Effect Effects)
public:
    enum Effect {
        None = 0,
        Wireframe = 1,
        Phong = 2
    };
    Q_DECLARE_FLAGS(Effects, Effect)


    enum EdgeType {
        HardEdge,
        SoftEdge
    };
    Q_ENUM(EdgeType)

    explicit CADMaterial(Qt3DCore::QNode *parent = nullptr);
    ~CADMaterial();

    QColor ambient() const;
    QColor diffuse() const;
    QColor specular() const;
    float shininess() const;
    EdgeType edgeType() const;
    float hardEdgeLimit() const;

    Effects materialEffects() const;

public Q_SLOTS:
    void setAmbient(const QColor &ambient);
    void setDiffuse(const QColor &diffuse);
    void setSpecular(const QColor &specular);
    void setShininess(float shininess);
    void setEdgeType(EdgeType edgeType);
    void setHardEdgeLimit(float hardEdgeLimit);
    void setMaterialEffects(const Effects &materialEffects);

Q_SIGNALS:
    void ambientChanged(const QColor &ambient);
    void diffuseChanged(const QColor &diffuse);
    void specularChanged(const QColor &specular);
    void shininessChanged(float shininess);
    void edgeTypeChanged(EdgeType edgeType);
    void hardEdgeLimitChanged(float hardEdgeLimit);
    void materialEffectsChanged(Effects materialEffects);

private:
    Qt3DRender::QEffect *m_cadEffect;

    Qt3DRender::QParameter *m_ambientParameter;
    Qt3DRender::QParameter *m_diffuseParameter;
    Qt3DRender::QParameter *m_specularParameter;
    Qt3DRender::QParameter *m_shininessParameter;
    Qt3DRender::QParameter *m_edgeTypeParameter;
    Qt3DRender::QParameter *m_hardEdgeLimitParameter;
    Qt3DRender::QParameter *m_lineWidthParameter;
    Qt3DRender::QParameter *m_lineColorParameter;
    Qt3DRender::QTechnique *m_cadGL3Technique;
    Qt3DRender::QRenderPass *m_phongGL3RenderPass;
    Qt3DRender::QRenderPass *m_wireframeGL3RenderPass;
    Qt3DRender::QDepthTest *m_wireframeDepthTest;
    Qt3DRender::QShaderProgram *m_phongGL3Shader;
    Qt3DRender::QShaderProgram *m_wireframeGL3Shader;

    Qt3DRender::QFilterKey *m_filterKey;

    Effects m_materialEffects;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(CADMaterial::Effects)

#endif // CADMATERIAL_H
