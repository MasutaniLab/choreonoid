/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneGraph>
#include <cnoid/SceneRenderer>
#include "exportdecl.h"

namespace cnoid {

class GLSceneRendererImpl;
    
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    GLSceneRenderer();
    GLSceneRenderer(SgGroup* root);
    virtual ~GLSceneRenderer();

    virtual void setOutputStream(std::ostream& os) = 0;

    virtual SgGroup* sceneRoot();
    virtual SgGroup* scene();
    virtual void clearScene();

    virtual bool initializeGL();
    virtual void flush();

    // The following functions cannot be called bofore calling the initializeGL() function.
    bool setSwapInterval(int interval);
    int getSwapInterval() const;

    virtual SignalProxy<void()> sigRenderingRequest();

    virtual int numCameras() const;
    virtual SgCamera* camera(int index);
    virtual const SgNodePath& cameraPath(int index) const;
    virtual SignalProxy<void()> sigCamerasChanged() const;
        
    virtual SgCamera* currentCamera() const;
    virtual int currentCameraIndex() const;
    virtual void setCurrentCamera(int index);
    virtual bool setCurrentCamera(SgCamera* camera);
    virtual SignalProxy<void()> sigCurrentCameraChanged();
    const Affine3& currentCameraPosition() const;

    virtual void setViewport(int x, int y, int width, int height);
    virtual Array4i viewport() const;
    void getViewport(int& out_x, int& out_y, int& out_width, int& out_height) const;
    virtual double aspectRatio() const; // width / height;

    void getViewFrustum(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const;
    void getViewVolume(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const;    

    const Vector3f& backgroundColor() const;
    void setBackgroundColor(const Vector3f& color);

    const Vector4f& defaultColor() const;
    void setDefaultColor(const Vector4f& color);

    virtual void setDefaultLighting(bool on) = 0;
    virtual SgLight* headLight();
    virtual void setHeadLight(SgLight* light);
    int numLights() const;
    void getLightInfo(int index, SgLight*& out_light, Affine3& out_position) const;
    void setAsDefaultLight(SgLight* light);
    void unsetDefaultLight(SgLight* light);
    void enableAdditionalLights(bool on);

    virtual void setDefaultSmoothShading(bool on) = 0;
    virtual SgMaterial* defaultMaterial() = 0;
    virtual void enableTexture(bool on) = 0;
    virtual void setDefaultPointSize(double size) = 0;
    virtual void setDefaultLineWidth(double width) = 0;

    enum PolygonMode { FILL_MODE, LINE_MODE, POINT_MODE };
    void setPolygonMode(PolygonMode mode);
    PolygonMode polygonMode() const;

    void enableFog(bool on);
    bool isFogEnabled() const;
    int numFogs() const;
    SgFog* fog(int index) const;

    virtual void showNormalVectors(double length) = 0;

    virtual void requestToClearCache() = 0;
    
    virtual bool pick(int x, int y) = 0;
    virtual const Vector3& pickedPoint() const = 0;
    virtual const SgNodePath& pickedNodePath() const = 0;

  protected:
    void extractPreproNodes();
    
    virtual void onImageUpdated(SgImage* image) = 0;

private:
    GLSceneRendererImpl* impl;
    friend class GLSceneRendererImpl;
};

}

#endif
