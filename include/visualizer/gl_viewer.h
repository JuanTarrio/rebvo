/******************************************************************************

   REBVO: RealTime Edge Based Visual Odometry For a Monocular Camera.
   Copyright (C) 2016  Juan José Tarrio

   Jose Tarrio, J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
   for a Monocular Camera. In Proceedings of the IEEE International Conference
   on Computer Vision (pp. 702-710).

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

 *******************************************************************************/

#ifndef GL_VIEWER_H
#define GL_VIEWER_H


#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>



#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>

#include "VideoLib/video_io.h"

#include "CommLib/net_keypoint.h"
#include "depth_filler.h"
#include "mtracklib/keyframe.h"

namespace rebvo {

struct RenderParams;
class gl_viewer;
class callbackFunc{

public:
    virtual void rederFunc(gl_viewer *gl,RenderParams*rp){}
};

struct RenderParams{
    net_keyline **net_kl=nullptr;
    int *net_kln=nullptr;
    int net_klistn;
    float zf;
    Point2DF pp;
    int net_kpn;
    std::vector<depth_filler> *d_filler=nullptr;
    bool draw_crash_cuad;
    std::vector<TooN::Vector<3> > *pos_tray=nullptr;
    TooN::Matrix<3,3> *Pose=nullptr;
    compressed_nav_pkg nav;
    int current_em;
    int total_em;
    TooN::Vector<4> ref_err;
    Image <RGB24Pixel> *img_data=nullptr;

    std::vector <keyframe> *kflist=nullptr;
    std::vector <bool> *kf_show_mask=nullptr;
    std::vector <callbackFunc*> renderCallbacks;
    bool render_match;
    double floor_dist=-1;
};


typedef union{
    struct {GLdouble ex,ey,ez,cx,cy,cz,ux,uy,uz;}s;
    GLdouble c[9];
    struct {GLdouble e[3],c[3],u[3];}g;
}CAMARA;

class CCam
{
private:
    CAMARA cam;
    double elev,azimut,lon;
    double xlma,xlmi,ylma,ylmi,zlma,zlmi;
public:
    void	Limites(double xmax,double xmin,double ymax,double ymin,double zmax,double zmin);
    void	Caminar(double x,double z);
    void	Volar(double y);
    void	Girar(double a,double e);
    void	glLookAt();
    CAMARA GetCam(){return cam;}
    void SetCam(CAMARA c){cam=c;}
    CCam(void);
    CCam(GLdouble ex,GLdouble ey,GLdouble ez,GLdouble cx,GLdouble cy,GLdouble cz,GLdouble ux,GLdouble uy,GLdouble uz);
    ~CCam(void);


    void    Reset(GLdouble ex,GLdouble ey,GLdouble ez,GLdouble cx,GLdouble cy,GLdouble cz,GLdouble ux,GLdouble uy,GLdouble uz);

    void    Save(const char *name);

    void    Load(const char *name);
};


class gl_viewer
{

    Display               * display;
    int                     screen;
    /* our window instance */
    Window                  window;
    GLXContext              context;
    XSetWindowAttributes    winAttr;
    bool                    doubleBuffered;
    int                     x, y;
    unsigned int            width, height;
    unsigned int            depth;

    float fov;

    CCam                    *cam;

    GLuint                  ImgTexture[2];

    RGB24Pixel              ColorGrad[512];

    cam_model               *cam_mod;

public:

    int                     RenderSurface;
    int                     RenderCuad;
    int                     RenderLines;
    bool                    RenderSigma;
    bool                    RenderTray;
    int                     FixView;


    double                  ColorZmin;
    double                  ColorZmax;

    bool                    RenderOcto;


public:
    gl_viewer(int width, int height, const char *title,float fov,cam_model *camera_model=nullptr);
    ~gl_viewer();
    void initGL();
    void resizeGL(int width, int height);

    void drawSphere(float x,float y,float z,float rx,float ry,float rz,int latn,int lonn, bool wired=true);
    void drawCube(bool wired=false);

    void drawQuad(float x, float y, float z, float r=0);
    void drawQuads(TooN::Vector<3> Vel, TooN::Vector<3> g_est, double time2impact, bool show_cc=false);

    void drawQuadsR(TooN::Vector<4> RefErr, TooN::Vector<3> g_est, double min_dist, bool show_cc=false, double floor_dist=-1);

    void drawTrayectory(const std::vector<TooN::Vector<3> > &pos_tray, const TooN::Matrix<3,3> &Pose);
    void fixView(const std::vector <TooN::Vector<3> > &pos_tray, const TooN::Matrix<3,3> &Pose);
    void fixViewG(const TooN::Vector<3> &est_g);
    void STR_View(const double &scale,const TooN::Vector<3> &pos, const TooN::Matrix<3,3> &Pose);
    void renderGL(RenderParams &rp);

    bool glDrawLoop(RenderParams &rp, bool ReRender, KeySym *key=nullptr);

    void drawKeyLines(net_keyline **net_kl, int *net_kln, int net_klistn, float zf, Point2DF &pp, bool tresh,bool DrawSigma);
    void drawFiller(depth_filler &df, Image<RGB24Pixel> *img_data=nullptr, double scale=1, bool use_optim_pnt=false);
    void drawFillerUnc(depth_filler &df, cam_model &cam, double scale, bool is_uper, float alpha);


    void drawKeyFrame(keyframe & kf, int render_mode, keyframe * kf_match, bool draw_unc, int draw_filler);

    void resetView();
    void translateView(float x,float y,float z);
    void rotateViewX(float a);
    void rotateViewY(float a);

    void Depth2Color(float z,float c[3]);



    void drawCamera(float x,float y,float z,float size);


    void SaveMatrixFile(const char *name);


    void LoadMatrixFile(const char *name);

    void InitTexture();
    void LoadTexture(Image<RGB24Pixel> &img_data);


    void ToggleFixView(int fv){FixView=fv%3;}
    void ToggleCameraView(int cv){RenderCuad=cv%3;}
    void fixViewGt(const TooN::Vector<3> &est_g);

    void setCameraModel(cam_model *camera_model){cam_mod=camera_model;}
private:
    void LoadTextureGradient();
    float Depth2Texture(float z);
    void initFont();
};
}
#endif // GL_VIEWER_H
