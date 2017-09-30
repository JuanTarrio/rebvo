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


#include <stdio.h>
#include <string.h>
#include <GL/freeglut.h>
#include <TooN/so3.h>

#include "visualizer/gl_viewer.h"

namespace rebvo {
/* attributes for a single buffered visual in RGBA format with at least
 * 4 bits per color and a 16 bit depth buffer */
static int attrListSgl[] =
{
    GLX_RGBA, GLX_RED_SIZE, 4,
    GLX_GREEN_SIZE, 4,
    GLX_BLUE_SIZE, 4,
    GLX_DEPTH_SIZE, 16,
    None
};

/* attributes for a double buffered visual in RGBA format with at least
 * 4 bits per color and a 16 bit depth buffer */
static int attrListDbl[] =
{
    GLX_RGBA, GLX_DOUBLEBUFFER,
    GLX_RED_SIZE, 4,
    GLX_GREEN_SIZE, 4,
    GLX_BLUE_SIZE, 4,
    GLX_DEPTH_SIZE, 16,
    None
};


CCam::CCam(void)
{
}

CCam::~CCam(void)
{
}

void CCam::Save(const char *name)
{
    FILE * f=fopen(name,"w");

    if(f==NULL){

        printf("\nGlview Camera: No puedo abrir archivo %s\n",name);
        return;
    }


    for(int i=0;i<9;i++)
        fprintf(f,"%e ",cam.c[i]);


    fprintf(f,"%e %e %e",elev,azimut,lon);

    fclose(f);


}

void CCam::Load(const char *name)
{
    FILE * f=fopen(name,"rb");

    if(f==NULL){
        printf("\nGlview: No puedo abrir archivo %s\n",name);
        return;
    }

    int r;
    for(int i=0;i<9;i++)
        r=fscanf(f,"%le",&cam.c[i]);


    r=fscanf(f,"%le %le %le",&elev,&azimut,&lon);

    fclose(f);


}

CCam::CCam(GLdouble ex,GLdouble ey,GLdouble ez,GLdouble cx,GLdouble cy,GLdouble cz,GLdouble ux,GLdouble uy,GLdouble uz){
    Reset(ex,ey,ez,cx,cy,cz,ux,uy,uz);
}


void CCam::Reset(GLdouble ex,GLdouble ey,GLdouble ez,GLdouble cx,GLdouble cy,GLdouble cz,GLdouble ux,GLdouble uy,GLdouble uz){
    cam.s.cx=cx;
    cam.s.cy=cy;
    cam.s.cz=cz;
    cam.s.ex=ex;
    cam.s.ey=ey;
    cam.s.ez=ez;
    cam.s.ux=ux;
    cam.s.uy=uy;
    cam.s.uz=uz;

    xlma=1e10;
    xlmi=-1e10;
    ylma=1e10;
    ylmi=-1e10;
    zlma=1e10;
    zlmi=-1e10;

    double dx=cam.s.cx-cam.s.ex,dz=cam.s.cz-cam.s.ez,dy=cam.s.cy-cam.s.ey;
    lon=pow(dx*dx+dy*dy+dz*dz,0.5);
    elev=asin(dy/lon);
    azimut=atan(dz/dx);
}


void CCam::Limites(double xmax,double xmin,double ymax,double ymin,double zmax,double zmin){
    xlma=xmax;
    xlmi=xmin;
    ylma=ymax;
    ylmi=ymin;
    zlma=zmax;
    zlmi=zmin;
}

void CCam::glLookAt(){
    gluLookAt (cam.s.ex, cam.s.ey, cam.s.ez, cam.s.cx, cam.s.cy, cam.s.cz,cam.s.ux, cam.s.uy, cam.s.uz);
}

void CCam::Caminar(double x,double z){
    double dx=cam.s.cx-cam.s.ex,dz=cam.s.cz-cam.s.ez,m,zbak,xbak;
    m=pow(dx*dx+dz*dz,0.5);
    dx/=m;dz/=m;
    xbak=cam.s.ex;zbak=cam.s.ez;
    cam.s.ex+=dx*x-dz*z;cam.s.ez+=dz*x+dx*z;
    if(cam.s.ex>xlma || cam.s.ex<xlmi || cam.s.ez>zlma || cam.s.ez<xlmi){
        cam.s.ex=xbak;cam.s.ez=zbak;
    }else{
        cam.s.cx+=dx*x-dz*z;cam.s.cz+=dz*x+dx*z;
    }

}



void CCam::Volar(double y){
    double ybak=cam.s.ey;
    cam.s.ey+=y;
    if(cam.s.ey>ylma || cam.s.ey<ylmi)
        cam.s.ey=ybak;
    else
        cam.s.cy+=y;
}

void CCam::Girar(double a,double e){
    azimut+=a;
    elev+=e;

    if(elev>M_PI/2 || elev<-M_PI/2)
        elev-=e;

    cam.s.cx=cam.s.ex+lon*cos(elev)*cos(azimut);
    cam.s.cy=cam.s.ey+lon*sin(elev);
    cam.s.cz=cam.s.ez+lon*cos(elev)*sin(azimut);
}

gl_viewer::~gl_viewer()
{
    if( context )
    {
        if( !glXMakeCurrent(display, None, NULL))
        {
            printf("Could not release drawing context.\n");
        }
        /* destroy the context */
        glXDestroyContext(display, context);
        context = NULL;
    }
    /* switch back to original desktop resolution if we were in fullscreen */

    XCloseDisplay(display);


}
gl_viewer::gl_viewer(int width, int height, const char *title, float fov, cam_model *camera_model)
    :cam_mod(camera_model)
{

    RenderSurface=0;
    RenderCuad=2;
    RenderLines=2;
    RenderSigma=false;
    ColorZmax=10;
    ColorZmin=0;
    RenderTray=false;
    FixView=false;

    this->width=width;
    this->height=height;
    this->fov=fov;

    XVisualInfo *vi;
    Colormap cmap;

    Atom wmDelete;

    cam=new CCam(0,0,0,0,0,-1.0,0,1.0,0);


    display = XOpenDisplay(0);
    screen = DefaultScreen(display);


    /* get an appropriate visual */
    vi = glXChooseVisual(display, screen, attrListDbl);
    if (vi == NULL)
    {
        vi = glXChooseVisual(display, screen, attrListSgl);
        doubleBuffered = False;
        printf("\nGLVIewer: singlebuffered rendering will be used, no doublebuffering available\n");
    }
    else
    {
        doubleBuffered = True;
        printf("\nGLVIewer: doublebuffered rendering available\n");
    }


    /* create a GLX context */
    context = glXCreateContext(display, vi, 0, GL_TRUE);

    /* create a color map */
    cmap = XCreateColormap(display, RootWindow(display, vi->screen),
                           vi->visual, AllocNone);

    winAttr.colormap = cmap;
    winAttr.border_pixel = 0;


    /* create a window in window mode*/
    winAttr.event_mask = ExposureMask | KeyPressMask | ButtonPressMask |
            StructureNotifyMask;

    window = XCreateWindow(display, RootWindow(display, vi->screen),
                           0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
                           CWBorderPixel | CWColormap | CWEventMask, &winAttr);

    /* only set window title and handle wm_delete_events if in windowed mode */
    wmDelete = XInternAtom(display, "WM_DELETE_WINDOW", True);
    XSetWMProtocols(display, window, &wmDelete, 1);
    XSetStandardProperties(display, window,title,title, None, NULL, 0, NULL);
    XMapRaised(display, window);

    /* connect the glx-context to the window */
    glXMakeCurrent(display, window, context);
    if (glXIsDirect(display, context))
        printf("\nGLVIewer: DRI enabled\n");
    else
        printf("\nGLVIewer: no DRI available\n");



   // initFont();
    initGL();
    InitTexture();

}


void gl_viewer::initFont(){
    const char *font_name="fixed";
    XFontStruct* font_info;
    int first;
    int last;

    GLuint font_base = glGenLists(256);
    if (!glIsList(font_base)) {
          std::cout<<"\nGlViewer: ny_init(): Out of display lists\n";
          return;
       }

    font_info = XLoadQueryFont(display, font_name);

    if (!font_info) {
       std::cout<<"\nGlViewer: XLoadQueryFont() failed.\n";
    }else{
        first = font_info->min_char_or_byte2;
        last  = font_info->max_char_or_byte2;
        glXUseXFont(font_info->fid, first, last-first+1, font_base+first);
    }
}

void gl_viewer::initGL(){


   // glEnable(GL_TEXTURE_2D);       /* Enable Texture Mapping */
   // glEnable(GL_TEXTURE_1D);       /* Enable Texture Mapping */
    glShadeModel(GL_SMOOTH);
    //glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);


    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat ligth_specular[] = { 0.0, 0.0, 0.0, 0.0};
    GLfloat ligth_diffuse[] = { 0.2, 0.2, 0.2, 0.0};
    GLfloat ligth_ambient[] = { 0.8, 0.8, 0.8, 1.0};
    GLfloat mat_shininess[] = {10.0 };

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

    glLightfv(GL_LIGHT0, GL_SPECULAR, ligth_specular);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, ligth_diffuse);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ligth_ambient);
    glEnable(GL_COLOR_MATERIAL);

    //glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    /* we use resizeGL once to set up our initial perspective */
    resizeGL(width, height);
    /* Reset the rotation angle of our object */
    glFlush();

}


void gl_viewer::resizeGL(int width, int height){
    if (height == 0)
        height = 1;
    glViewport(0, 0, width, height);

    resetView();

   // this->width=width;
  //  this->height=height;
}


void gl_viewer::InitTexture(){

//    glGenTextures(2, ImgTexture);   /* create the texture */
//    glBindTexture(GL_TEXTURE_2D, ImgTexture[0]);
//    glBindTexture(GL_TEXTURE_1D, ImgTexture[1]);

    for(int i=0;i<256;i++){
        ColorGrad[i].pix.r=255-i;
        ColorGrad[i].pix.g=i;
        ColorGrad[i].pix.b=0;
    }
    for(int i=256;i<512;i++){
        ColorGrad[i].pix.r=0;
        ColorGrad[i].pix.g=511-i;
        ColorGrad[i].pix.b=i-256;
    }
    glTexImage1D(GL_TEXTURE_1D, 0, 3, 512, 0,GL_RGB, GL_UNSIGNED_BYTE, ColorGrad);
    /* enable linear filtering */
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


}

void gl_viewer::LoadTexture(Image <RGB24Pixel> &img_data){


   // glBindTexture(GL_TEXTURE_2D, ImgTexture[0]);
    /* actually generate the texture */
    glTexImage2D(GL_TEXTURE_2D, 0, 3, img_data.Size().w, img_data.Size().h, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, img_data.Data());
    /* enable linear filtering */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


}

void gl_viewer::LoadTextureGradient(){


    //glBindTexture(GL_TEXTURE_1D, ImgTexture[1]);
    /* actually generate the texture */


}

float gl_viewer::Depth2Texture(float z){

    return util::Constrain((ColorZmax-z)/(ColorZmax-ColorZmin),0.001,0.999);

}


void gl_viewer::drawFiller(depth_filler &df, Image <RGB24Pixel> *img_data, double scale,bool use_optim_pnt){


    glPushMatrix();

    if(img_data){
        glEnable(GL_TEXTURE_2D);
        LoadTexture(*img_data);
    }else{
        glEnable(GL_TEXTURE_1D);
        LoadTextureGradient();
    }

    glBegin(GL_TRIANGLES);

    for(float y=df.blockSize().h/2.0;y<df.imageSize().h-df.blockSize().h;y+=df.blockSize().h){
        for(float x=df.blockSize().w/2.0;x<df.imageSize().w-df.blockSize().w;x+=df.blockSize().w){

            if(!df.IsImgVisibleSimple(x,y))
                continue;

            TooN::Vector <3> P00,P10,P01,P11;
            TooN::Vector <3> N00,N10,N01,N11;
            if(use_optim_pnt){
                P00=df.getImgPoint(x,y).optim_point;
                P10=df.getImgPoint(x,y+df.blockSize().h).optim_point;
                P01=df.getImgPoint(x+df.blockSize().w,y).optim_point;
                P11=df.getImgPoint(x+df.blockSize().w,y+df.blockSize().h).optim_point;


                N00=df.getImgPoint(x,y).normal;
                N10=df.getImgPoint(x,y+df.blockSize().h).normal;
                N01=df.getImgPoint(x+df.blockSize().w,y).normal;
                N11=df.getImgPoint(x+df.blockSize().w,y+df.blockSize().h).normal;


            }else{
                P00=df.getImg3DPos(x,y);
                P10=df.getImg3DPos(x,y+df.blockSize().h);
                P01=df.getImg3DPos(x+df.blockSize().w,y);
                P11=df.getImg3DPos(x+df.blockSize().w,y+df.blockSize().h);


                N00=df.getImgPoint(x,y).normal;
                N10=df.getImgPoint(x,y+df.blockSize().h).normal;
                N01=df.getImgPoint(x+df.blockSize().w,y).normal;
                N11=df.getImgPoint(x+df.blockSize().w,y+df.blockSize().h).normal;
            }

            if(img_data){

                glColor3f(1.0, 1.0, 1.0);


                glTexCoord2f((float)x/(float)img_data->Size().w,(float)y/(float)img_data->Size().h);
                glVertex3f(P00[0], P00[1], P00[2]);
                glTexCoord2f((float)(x+df.blockSize().w)/(float)img_data->Size().w,(float)y/(float)img_data->Size().h);
                glVertex3f(P01[0], P01[1], P01[2]);
                glTexCoord2f((float)(x+df.blockSize().w)/(float)img_data->Size().w,(float)(y+df.blockSize().h)/(float)img_data->Size().h);
                glVertex3f(P11[0], P11[1], P11[2]);

                glTexCoord2f((float)x/(float)img_data->Size().w,(float)y/(float)img_data->Size().h);
                glVertex3f(P00[0], P00[1], P00[2]);
                glTexCoord2f((float)x/(float)img_data->Size().w,(float)(y+df.blockSize().h)/(float)img_data->Size().h);
                glVertex3f(P10[0], P10[1], P10[2]);
                glTexCoord2f((float)(x+df.blockSize().w)/(float)img_data->Size().w,(float)(y+df.blockSize().h)/(float)img_data->Size().h);
                glVertex3f(P11[0], P11[1], P11[2]);


            }else{

                glColor3f(1.0, 1.0, 1.0);

            /*    Depth2Color(df.getImgPoint(x,y).dist*scale,color);
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
                glColor3f(color[0], color[1], color[2]);
*/

                glTexCoord1f(Depth2Texture(df.getImgPoint(x,y).dist*scale));
                //std::cout << Depth2Texture(df.getImgPoint(x,y).dist*scale)<<" ";
                glNormal3f(N00[0], N00[1], N00[2]);
                glVertex3f(P00[0], P00[1], P00[2]);


                glTexCoord1f(Depth2Texture(df.getImgPoint(x+df.blockSize().w,y).dist*scale));
                glNormal3f(N01[0], N01[1], N01[2]);
                glVertex3f(P01[0], P01[1], P01[2]);


                glTexCoord1f(Depth2Texture(df.getImgPoint(x+df.blockSize().w,y+df.blockSize().h).dist*scale));
                glNormal3f(N11[0], N11[1], N11[2]);
                glVertex3f(P11[0], P11[1], P11[2]);


                glTexCoord1f(Depth2Texture(df.getImgPoint(x,y).dist*scale));
                glNormal3f(N00[0], N00[1], N00[2]);
                glVertex3f(P00[0], P00[1], P00[2]);


                glTexCoord1f(Depth2Texture(df.getImgPoint(x,y+df.blockSize().h).dist*scale));
                glNormal3f(N10[0], N10[1], N10[2]);
                glVertex3f(P10[0], P10[1], P10[2]);


                glTexCoord1f(Depth2Texture(df.getImgPoint(x+df.blockSize().w,y+df.blockSize().h).dist*scale));
                glNormal3f(N11[0], N11[1], N11[2]);
                glVertex3f(P11[0], P11[1], P11[2]);

            }






        }
    }


    glEnd();

    if(img_data){
        glDisable(GL_TEXTURE_2D);

    }else{
        glDisable(GL_TEXTURE_1D);
    }
    glPopMatrix();
    
}


void gl_viewer::drawFillerUnc(depth_filler &df, cam_model &cam, double scale, bool is_uper,float alpha){




        glBegin(GL_LINES);

        for(int y=0;y<=df.imageSize().h-df.blockSize().h;y+=df.blockSize().h){
            for(int x=0;x<=df.imageSize().w-df.blockSize().w;x+=df.blockSize().w){

                if(!df.IsImgVisible(x,y))
                    continue;


                double srho00,srho10,srho01,srho11;
                double rho00=df.getImgRho(x,y,&srho00);
                double rho10=df.getImgRho(x,y+df.blockSize().h,&srho10);
                double rho01=df.getImgRho(x+df.blockSize().w,y,&srho01);
                double rho11=df.getImgRho(x+df.blockSize().w,y+df.blockSize().h,&srho11);

                if(is_uper){
                    rho00+=srho00;
                    rho01+=srho01;
                    rho10+=srho10;
                    rho11+=srho11;
                }else{
                    rho00-=srho00;
                    rho01-=srho01;
                    rho10-=srho10;
                    rho11-=srho11;
                }

                rho00=util::Constrain(rho00,RHO_MIN,RHO_MAX);
                rho01=util::Constrain(rho01,RHO_MIN,RHO_MAX);
                rho10=util::Constrain(rho10,RHO_MIN,RHO_MAX);
                rho11=util::Constrain(rho11,RHO_MIN,RHO_MAX);

                Point3D<float> P00=cam.unprojectImgCord(Point3D<float>((double)x,(double)y,rho00));
                Point3D<float> P10=cam.unprojectImgCord(Point3D<float>((double)x,(double)(y+df.blockSize().h),rho10));
                Point3D<float> P01=cam.unprojectImgCord(Point3D<float>((double)(x+df.blockSize().w),(double)y,rho01));
                Point3D<float> P11=cam.unprojectImgCord(Point3D<float>((double)(x+df.blockSize().w),(double)(y+df.blockSize().h),rho11));



                float color[3];

                Depth2Color(std::min(std::min(df.GetImgDist(x,y),df.GetImgDist(x+1,y)),std::min(df.GetImgDist(x,y+1),df.GetImgDist(x+1,y+1)))*scale,color);

                if(!is_uper){
                    glColor4f(130.0/255.0, 235.0/255.0, 255.0/255.0,alpha);
                }else{
                    glColor4f(184.0/255.0, 0, 198.0/255.0,alpha);
                }

                glVertex3f(P00.x, P00.y, P00.z);
                glVertex3f(P01.x, P01.y, P01.z);
                glVertex3f(P01.x, P01.y, P01.z);
                glVertex3f(P11.x, P11.y, P11.z);
                glVertex3f(P11.x, P11.y, P11.z);
                glVertex3f(P10.x, P10.y, P10.z);
                glVertex3f(P10.x, P10.y, P10.z);
                glVertex3f(P00.x, P00.y, P00.z);



            }
        }


        glEnd();






}


void gl_viewer::drawKeyLines(net_keyline **net_kl, int *net_kln, int net_klistn, float zf, Point2DF &pp, bool tresh, bool DrawSigma)
{


    glDepthFunc(GL_ALWAYS);

    glBegin(GL_LINES);

    //glBegin(GL_QUADS);



    for(int j=0;j<net_klistn;j++){

        for(int i=0;i<net_kln[j];i++){



            float x1,x2,y1,y2,z1,z2,c,dz1,dz2;

            bool connected=false;


            c=(float)net_kl[j][i].s_rho/NET_RHO_SCALING;

            c=1/(c*c);

            if(net_kl[j][i].n_kl>=0){


                z1=NET_RHO_SCALING/net_kl[j][i].rho;
                dz1=z1-NET_RHO_SCALING/(net_kl[j][i].rho+(float)net_kl[j][i].s_rho);


                x1=((float)net_kl[j][i].qx-(float)pp.x)*z1/zf;
                y1=((float)net_kl[j][i].qy-(float)pp.y)*z1/zf;

                int nkl=net_kl[j][i].n_kl;


                z2=NET_RHO_SCALING/net_kl[0][nkl].rho;
                dz2=z2-NET_RHO_SCALING/(net_kl[0][nkl].rho+(float)net_kl[0][nkl].s_rho);


                z2=NET_RHO_SCALING/net_kl[0][nkl].rho;
                dz2=z2-NET_RHO_SCALING/(net_kl[0][nkl].rho+(float)net_kl[0][nkl].s_rho);

                x2=((float)net_kl[0][nkl].qx+0.0-(float)pp.x)*z2/zf;
                y2=((float)net_kl[0][nkl].qy-(float)pp.y)*z2/zf;

                if(fabs(z1-z2)<std::min(dz1,dz2)){
                    connected=true;

                }


            }else{
                continue;
            }


            if(tresh){
                if(c<20 || !connected)
                    continue;
                float c3[3];
                Depth2Color((z1+z2)/2,c3);

                glColor3f(c3[0],c3[1],c3[2]);
            }else{
                //c/=25.5;
                c=(float)net_kl[j][i].rho/(float)net_kl[j][i].s_rho;
                util::keep_min(c,1.0);

                if(connected)
                    glColor3f(c, 0, 1-c);
                else
                    glColor3f(c, 1-c, 1-c);
            }



            //    if(dz1 < z1 && dz2 < z2){

            glVertex3f(x1, y1, z1);
            glVertex3f(x2, y2, z2);

            if(DrawSigma && dz1 < z1 && dz2 < z2){
                glColor3f(0, 0.5, 0);


                z1=NET_RHO_SCALING/(net_kl[j][i].rho+net_kl[j][i].s_rho);
                x1=((float)net_kl[j][i].qx-(float)pp.x)*z1/zf;
                y1=((float)net_kl[j][i].qy-(float)pp.y)*z1/zf;

                glVertex3f(x1, y1, z1);

                z1=NET_RHO_SCALING/std::max((int)net_kl[j][i].rho-(int)net_kl[j][i].s_rho,1);
                x1=((float)net_kl[j][i].qx-(float)pp.x)*z1/zf;
                y1=((float)net_kl[j][i].qy-(float)pp.y)*z1/zf;

                glVertex3f(x1, y1, z1);
            }
            //}

        }
    }

    glEnd();


    glDepthFunc(GL_LEQUAL);


}



void gl_viewer::drawKeyFrame(keyframe & kf, int render_mode,keyframe * kf_match,bool draw_unc,int draw_filler)
{


    glPushMatrix();
    STR_View(kf.K,kf.Pos,kf.Pose);

    glDepthFunc(GL_ALWAYS);



    //glBegin(GL_QUADS);

    TooN::Matrix <3,3>DPose=TooN::Identity;
    TooN::Vector <3>DPos=TooN::Zeros;

    if(kf_match){
        DPose=kf.Pose.T()*kf_match->Pose;
        DPos=kf.Pose.T()*(kf_match->Pos-kf.Pos);
    }

    if(render_mode>0){


        glBegin(GL_LINES);

        for(int kli=0;kli<kf.edges().KNum();kli++){
            KeyLine &kl=kf.edges()[kli];

            if(kl.n_id<0 ) //|| ( kl.m_id_kf<0 && kl.m_id_f<0)
                continue;

            float x1,x2,y1,y2,z1,z2,c,dz1,dz2;

            bool connected=false;

            c=kl.s_rho;
            c=1/(c*c);

            z1=1/kl.rho;
            dz1=z1-1/(kl.rho+kl.s_rho);

            x1=(kl.p_m.x)*z1/kf.camera.zfm;
            y1=(kl.p_m.y)*z1/kf.camera.zfm;


            KeyLine &kl2=(kf.edges())[kl.n_id];

            z2=1/kl2.rho;
            dz2=z2-1/(kl2.rho+kl2.s_rho);


            x2=(kl2.p_m.x)*z2/kf.camera.zfm;
            y2=(kl2.p_m.y)*z2/kf.camera.zfm;

            if(fabs(z1-z2)<std::min(dz1,dz2)){
                connected=true;
            }

            if(render_mode==2){
                if(c<20 || !connected)
                    continue;
                float c3[3];
                Depth2Color((z1+z2)/2,c3);

                glColor3f(c3[0],c3[1],c3[2]);
            }else{
                //c/=25.5;
                c=kl.rho/kl.s_rho;
                util::keep_min(c,1.0);


                if(connected)
                    glColor3f(c, 0, 1-c);
                else
                    glColor3f(c, 1-c, 1-c);
            }


            glVertex3f(x1, y1, z1);
            glVertex3f(x2, y2, z2);

            if(draw_unc){
                glColor3f(1, 1, 0);

                if(kl.rho<kl.s_rho){
                    glColor4f(0.1, 0.1, 0,0.1);
                }

                    z1=1/(kl.rho+kl.s_rho);
                    x1=(kl.p_m.x)*z1/kf.camera.zfm;
                    y1=(kl.p_m.y)*z1/kf.camera.zfm;
                    glVertex3f(x1, y1, z1);


                    z1=1/std::max(kl.rho-kl.s_rho,1e-3);
                    x1=(kl.p_m.x)*z1/kf.camera.zfm;
                    y1=(kl.p_m.y)*z1/kf.camera.zfm;
                    glVertex3f(x1, y1, z1);



            }
            /*
        if(kf_match && kl.kf_id>=0){



            z2=1/kf_match->edges()[kl.kf_id].rho;
            x2=(kf_match->edges()[kl.kf_id].p_m.x)*z2/kf_match->camera.zfm;
            y2=(kf_match->edges()[kl.kf_id].p_m.y)*z2/kf_match->camera.zfm;

            glColor3f(0, 1, 0);
            glVertex3f(x1, y1, z1);
            TooN::Vector<3>p2=DPose*TooN::makeVector(x2,y2,z2)+DPos;
            glVertex3f(p2[0],p2[1],p2[2]);

        }
        */
        }

        glEnd();

    }


    glDepthFunc(GL_LEQUAL);

    if(draw_filler>0 && kf.depthFillerAval()){
        drawFiller(kf.depthFill(),draw_filler>1?kf.depthFill().getImgc():nullptr,kf.K,false);

        if(draw_unc){

            drawFillerUnc(kf.depthFill(),kf.camera,kf.K,true,0.5);
            drawFillerUnc(kf.depthFill(),kf.camera,kf.K,false,0.5);
        }
    }



    glPopMatrix();


    glPushMatrix();
    STR_View(1,kf.Pos,kf.Pose);
    drawCamera(0,0,0,0.05);
    glPopMatrix();

}


void gl_viewer::Depth2Color(float z,float c[3]){




    float r=util::Constrain((z-ColorZmin)/(ColorZmax-ColorZmin),0.001,0.999);

    if(r>0.5){

        r=(r-0.5)*2;
        c[0]=r;
        c[1]=1-r;
        c[2]=0;

    }else{


        r*=2;
        c[0]=0;
        c[1]=r;
        c[2]=1-r;
    }



}

void gl_viewer::drawQuad(float x,float y,float z,float r){


    glPushMatrix();
    glTranslatef(x,y,z);

    glRotatef(r*180/M_PI,0,1,0);

    glPushMatrix();
    glScalef(0.02,0.02,0.4);
    drawCube();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,0,0.2);
    glScalef(0.4,0.02,0.02);
    drawCube();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,0,-0.2);
    glScalef(0.4,0.02,0.02);
    drawCube();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,-0.03,-0.021);
    glScalef(0.02,0.04,0.04);
    drawCube();
    glPopMatrix();

    glPopMatrix();
}


void gl_viewer::drawCamera(float x,float y,float z,float size){


    glPushMatrix();
    glTranslatef(x,y,z);


    glBegin(GL_LINES);

    glColor3f(1,1,1);

    float v_size=size*tan(fov*M_PI/360);
    float h_size=v_size*width/height;


    glVertex3f(0,0,0);
    glVertex3f(h_size,v_size,size);
    glVertex3f(0,0,0);
    glVertex3f(-h_size,v_size,size);
    glVertex3f(0,0,0);
    glVertex3f(h_size,-v_size,size);
    glVertex3f(0,0,0);
    glVertex3f(-h_size,-v_size,size);


    glVertex3f(h_size,v_size,size);
    glVertex3f(h_size,-v_size,size);

    glVertex3f(h_size,-v_size,size);
    glVertex3f(-h_size,-v_size,size);

    glVertex3f(-h_size,-v_size,size);
    glVertex3f(-h_size,v_size,size);

    glVertex3f(-h_size,v_size,size);
    glVertex3f(h_size,v_size,size);

    glEnd();

    glPopMatrix();
}


void gl_viewer::drawQuads(TooN::Vector<3> Vel, TooN::Vector<3> g_est, double time2impact, bool show_cc){



    const double t2i_sat=20;




    glBegin(GL_LINES);
    float c=1;

    util::keep_min(time2impact,t2i_sat);

    c=time2impact/t2i_sat;


    glColor3f((1-c),c,0);

    glVertex3f(0, 0, 0);
    glVertex3f(Vel[0]*time2impact, Vel[1]*time2impact, Vel[2]*time2impact);

    glEnd();

    if(show_cc && time2impact<t2i_sat){
        glColor4f((1-c),c,0,0.4);
        drawQuad(Vel[0]*time2impact, Vel[1]*time2impact+0.03, Vel[2]*time2impact);

        glColor4f((1-c),c,0,0.2);
        drawSphere(Vel[0]*time2impact, Vel[1]*time2impact, Vel[2]*time2impact,0.4,0.4,0.4,20,20,true);
    }

    glColor3f((1-c),c,0);

    drawQuad(0,0.03,0);

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0, 0, 0);
    glVertex3f(g_est[0], g_est[1], g_est[2]);

    glEnd();



}


void gl_viewer::drawQuadsR(TooN::Vector<4> RefErr, TooN::Vector<3> g_est,double min_dist, bool show_cc,double floor_dist){



    const double red_dist=0.25;

    float c=red_dist/std::max(min_dist,red_dist);



    if(show_cc && TooN::norm(RefErr)>1e-10){
        glPushMatrix();

        fixViewGt(g_est);
        glBegin(GL_LINES);
        glColor3f(c,0,1-c);
        glVertex3f(0, 0, 0);
        glVertex3f(RefErr[2], -RefErr[0], RefErr[1]);
        glEnd();


        glTranslatef(RefErr[2], -RefErr[0]+0.03, RefErr[1]);

        glColor4f(c,0,1-c,0.4);
        drawQuad(0,0,0,RefErr[3]);

        glPopMatrix();
        //glColor4f((1-c),c,0,0.2);
        //drawSphere(-RefErr[2], -RefErr[0]+0.03, -RefErr[1],0.4,0.4,0.4,20,20,true);
    }


    glColor3f(c,1-c,0);

    drawQuad(0,0.03,0);

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0, 0, 0);
    g_est/=20;
    glVertex3f(g_est[0], g_est[1], g_est[2]);

    glEnd();


    if(floor_dist>0){
        TooN::Vector<3> u_g=TooN::unit(TooN::makeVector(g_est[0], g_est[1], g_est[2]));
        TooN::Vector<3> u_p1=TooN::makeVector(1, 0, 0)^u_g;
        TooN::Vector<3> u_p2=u_p1^u_g;

        u_p1*=0.25;
        u_p2*=0.25;

        TooN::Vector<3> pp0=floor_dist*u_g+u_p1+u_p2;
        TooN::Vector<3> pp1=floor_dist*u_g+u_p1-u_p2;
        TooN::Vector<3> pp2=floor_dist*u_g-u_p1-u_p2;
        TooN::Vector<3> pp3=floor_dist*u_g-u_p1+u_p2;

        glColor3f(228.0/255.0,229.0/255.0,103.0/255.0);
        glBegin(GL_LINES);

        glVertex3f(pp0[0], pp0[1], pp0[2]);
        glVertex3f(pp1[0], pp1[1], pp1[2]);
        glVertex3f(pp1[0], pp1[1], pp1[2]);
        glVertex3f(pp2[0], pp2[1], pp2[2]);
        glVertex3f(pp2[0], pp2[1], pp2[2]);
        glVertex3f(pp3[0], pp3[1], pp3[2]);
        glVertex3f(pp3[0], pp3[1], pp3[2]);
        glVertex3f(pp0[0], pp0[1], pp0[2]);


        pp0=floor_dist*u_g-u_p1;
        pp1=floor_dist*u_g+u_p1;
        glVertex3f(pp0[0], pp0[1], pp0[2]);
        glVertex3f(pp1[0], pp1[1], pp1[2]);

        pp0=floor_dist*u_g-u_p2;
        pp1=floor_dist*u_g+u_p2;
        glVertex3f(pp0[0], pp0[1], pp0[2]);
        glVertex3f(pp1[0], pp1[1], pp1[2]);

        glEnd();

    }



}

void gl_viewer::drawTrayectory(const std::vector <TooN::Vector<3> > &pos_tray, const TooN::Matrix<3, 3> &Pose){


    glBegin(GL_LINES);

    glColor3f(0xFF,0xFF,0);

    if(pos_tray.size()>1){
        TooN::Vector <3> pf=pos_tray[pos_tray.size()-1];
        for(int i=0;i<(int)pos_tray.size()-1;i++){

            TooN::Vector <3> p0=-(Pose.T()*(pf-pos_tray[i]));
            TooN::Vector <3> p1=-(Pose.T()*(pf-pos_tray[i+1]));

            glVertex3f(p0[0],p0[1],p0[2]);
            glVertex3f(p1[0],p1[1],p1[2]);
        }
    }

    glEnd();


}


void gl_viewer::fixView(const std::vector <TooN::Vector<3> > &pos_tray, const TooN::Matrix<3,3> &Pose){
    TooN::Vector <3> pos=pos_tray[pos_tray.size()-1];
    double matrix[16];
    for(int i=0;i<16;i++)
        matrix[i]=0;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            matrix[j*4+i]=Pose(i,j);
    matrix[15]=1;
    glTranslatef(pos[0],pos[1],pos[2]);
    glMultMatrixd(matrix);
}

void gl_viewer::fixViewG(const TooN::Vector<3> &est_g){


    using namespace TooN;

    if(norm(est_g)>0){

        Vector <3> u_y=makeVector(0,1,0);
        SO3 <>rot(est_g,u_y);


        Vector <3> u_z=rot*makeVector(0,0,1);

        SO3 <>rot2(makeVector(u_z[0],0,u_z[2]),makeVector(0,0,1));

        const Matrix<3,3> &Pose=rot2.get_matrix()*rot.get_matrix();

        double matrix[16];
        for(int i=0;i<16;i++)
            matrix[i]=0;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                matrix[j*4+i]=Pose(i,j);
        matrix[15]=1;

        glMultMatrixd(matrix);
    }
}

void gl_viewer::fixViewGt(const TooN::Vector<3> &est_g){


    using namespace TooN;

    if(norm(est_g)>0){

        Vector <3> u_y=makeVector(0,1,0);
        SO3 <>rot(est_g,u_y);


        Vector <3> u_z=rot*makeVector(0,0,1);

        SO3 <>rot2(makeVector(u_z[0],0,u_z[2]),makeVector(0,0,1));

        const Matrix<3,3> &Pose=rot2.get_matrix()*rot.get_matrix();

        double matrix[16];
        for(int i=0;i<16;i++)
            matrix[i]=0;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                matrix[j*4+i]=Pose(j,i);
        matrix[15]=1;

        glMultMatrixd(matrix);
    }
}

void gl_viewer::STR_View(const double &scale,const TooN::Vector<3> &pos, const TooN::Matrix<3,3> &Pose){
    double matrix[16];
    for(int i=0;i<16;i++)
        matrix[i]=0;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            matrix[j*4+i]=Pose(i,j);
    matrix[15]=1;

    glTranslatef(pos[0],pos[1],pos[2]);
    glMultMatrixd(matrix);
    glScaled(scale,scale,scale);
}

void gl_viewer::renderGL(RenderParams &rp)
{

    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);




    glPushMatrix();

    if(FixView==2 &&rp.pos_tray &&rp.Pose && rp.pos_tray->size()>0){
        fixView(*rp.pos_tray,*rp.Pose);
    }else if(FixView==1){
        fixViewG(rp.nav.G);
    }


    GLfloat light_position[] = { 0.0, 0.0, 3.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

     if(RenderSurface>0 && rp.d_filler)
         for(int i=0;i<(*rp.d_filler).size();i++){
             drawFiller((*rp.d_filler)[i],RenderSurface==2?(nullptr):(rp.img_data));
         }


    if(RenderLines>0 && rp.net_kl && rp.net_kln)
        drawKeyLines(rp.net_kl,rp.net_kln,rp.net_klistn,rp.zf,rp.pp,RenderLines==2,RenderSigma);


    if(rp.kflist)
        for(int i=0;i<rp.kflist->size();i++)
            if((*rp.kf_show_mask)[i]){
                if(rp.render_match && i>0 && (*rp.kf_show_mask)[i-1]){
                    drawKeyFrame((*rp.kflist)[i],RenderLines,&(*rp.kflist)[i-1],RenderSigma,RenderSurface);
                }else{
                    drawKeyFrame((*rp.kflist)[i],RenderLines,nullptr,RenderSigma,RenderSurface);

                }
            }


    switch(RenderCuad){
    case 1:
        //drawQuads(rp.c_det->V,rp.em_comp[rp.current_em].GetGEst()/20,rp.c_det->time2impact,rp.draw_crash_cuad);
    {
        double min_dist=1e20;
        if(rp.d_filler && (*rp.d_filler).size()>0)
            min_dist=((*rp.d_filler)[0]).GetMinDist();
        drawQuadsR(rp.ref_err,rp.nav.G,min_dist,rp.draw_crash_cuad,rp.floor_dist);
    }
        break;
    case 2:
        drawCamera(0,0,0,0.05);
        break;
    }

    if(RenderTray)
        drawTrayectory(*rp.pos_tray,*rp.Pose);


    for(callbackFunc* &rf:rp.renderCallbacks){
        rf->rederFunc(this,&rp);
    }


    glPopMatrix();



    /* swap the buffers if we have doublebuffered */
    if (doubleBuffered)
    {
        glXSwapBuffers(display, window);
    }
}


void gl_viewer::resetView(){

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if(cam_mod){

        double far=1.0/RHO_MIN;
        double near=1.0/RHO_MAX;
        double fx=cam_mod->zfm;
        double fy=cam_mod->zfm;
        double w=cam_mod->sz.w;
        double h=cam_mod->sz.h;
        double cx=cam_mod->pp.x;
        double cy=cam_mod->pp.y;



        //GLdouble perspMatrix[16]={2*fx/w,0,0,0,0,2*fy/h,0,0,2*(cx/w)-1,2*(cy/h)-1,(far+near)/(far-near),1,0,0,-2*far*near/(far-near),0};

        GLdouble perspMatrix[16]={2*fx/w,0,0,0,0,2*fy/h,0,0,2*(cx/w)-1,2*(cy/h)-1,-(far+near)/(far-near),-1,0,0,-2*far*near/(far-near),0};

        glMultMatrixd(perspMatrix);


        cam->glLookAt();
        glRotatef(180.0, 1.0, 0.0, 0.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        //
    }else{
        gluPerspective(fov, (GLfloat)width / (GLfloat)height, 1.0/RHO_MAX, 1.0/RHO_MIN);


        cam->glLookAt();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(180.0, 1.0, 0.0, 0.0);
    }




}


void gl_viewer::translateView(float x,float y,float z){


    cam->Caminar(-z,-x);
    cam->Volar(y);
    resetView();
}

void gl_viewer::rotateViewX(float a){

    cam->Girar(0,-a*M_PI/180.0);
    resetView();
}

void gl_viewer::rotateViewY(float a){


    cam->Girar(-a*M_PI/180.0,0);
    resetView();
}


void gl_viewer::drawCube(bool wired){


    if(wired)
        glBegin(GL_LINES);
    else
        glBegin(GL_QUADS);


    glVertex3f(-0.5,0.5,-0.5);
    glVertex3f(-0.5,0.5, 0.5);
    glVertex3f( 0.5,0.5, 0.5);
    glVertex3f( 0.5,0.5,-0.5);


    glVertex3f(-0.5,-0.5,-0.5);
    glVertex3f(-0.5,-0.5, 0.5);
    glVertex3f( 0.5,-0.5, 0.5);
    glVertex3f( 0.5,-0.5,-0.5);


    glVertex3f(0.5,-0.5,-0.5);
    glVertex3f(0.5,-0.5, 0.5);
    glVertex3f(0.5, 0.5, 0.5);
    glVertex3f(0.5, 0.5,-0.5);


    glVertex3f(-0.5,-0.5,-0.5);
    glVertex3f(-0.5,-0.5, 0.5);
    glVertex3f(-0.5, 0.5, 0.5);
    glVertex3f(-0.5, 0.5,-0.5);


    glVertex3f(-0.5,-0.5,0.5);
    glVertex3f(-0.5, 0.5,0.5);
    glVertex3f( 0.5, 0.5,0.5);
    glVertex3f( 0.5,-0.5,0.5);

    glVertex3f(-0.5,-0.5,-0.5);
    glVertex3f(-0.5, 0.5,-0.5);
    glVertex3f( 0.5, 0.5,-0.5);
    glVertex3f( 0.5,-0.5,-0.5);

    glEnd();

}

void gl_viewer::drawSphere(float x,float y,float z,float rx,float ry,float rz,int latn,int lonn,bool wired){


    glPushMatrix();

    glTranslatef(x,y,z);
    glScalef(rx,ry,rz);

    if(wired)
        glBegin(GL_LINES);
    else
        glBegin(GL_QUADS);

    for(int i=0;i<latn;i++){

        float ylat1=cos((float)i/(float)latn*M_PI);
        float ylat2=cos((float)(i+1)/(float)latn*M_PI);


        float rlat1=sin((float)i/(float)latn*M_PI);
        float rlat2=sin((float)(i+1)/(float)latn*M_PI);

        for(int j=0;j<lonn;j++){

            float xlon_r1_1=rlat1*cos((float)j/(float)latn*2*M_PI);
            float xlon_r1_2=rlat1*cos((float)(j+1)/(float)latn*2*M_PI);

            float xlon_r2_1=rlat2*cos((float)j/(float)latn*2*M_PI);
            float xlon_r2_2=rlat2*cos((float)(j+1)/(float)latn*2*M_PI);

            float zlon_r1_1=rlat1*sin((float)j/(float)latn*2*M_PI);
            float zlon_r1_2=rlat1*sin((float)(j+1)/(float)latn*2*M_PI);

            float zlon_r2_1=rlat2*sin((float)j/(float)latn*2*M_PI);
            float zlon_r2_2=rlat2*sin((float)(j+1)/(float)latn*2*M_PI);

            glVertex3f(xlon_r1_1,ylat1,zlon_r1_1);
            if(wired)
                glVertex3f(xlon_r1_2,ylat1,zlon_r1_2);

            glVertex3f(xlon_r1_2,ylat1,zlon_r1_2);
            if(wired)
                glVertex3f(xlon_r2_2,ylat2,zlon_r2_2);

            glVertex3f(xlon_r2_2,ylat2,zlon_r2_2);
            if(wired)
                glVertex3f(xlon_r2_1,ylat2,zlon_r2_1);

            glVertex3f(xlon_r2_1,ylat2,zlon_r2_1);
            if(wired)
                glVertex3f(xlon_r1_1,ylat1,zlon_r1_1);

        }

    }

    glEnd();


    glPopMatrix();
}



bool gl_viewer::glDrawLoop(RenderParams &rp,bool ReRender,KeySym *key){


    XEvent event;
    bool current=false;

    while (XCheckWindowEvent(display,window,winAttr.event_mask,&event))
    {

        glXMakeCurrent(display, window, context);
        current=true;
        switch (event.type)
        {
        case Expose:
            if (event.xexpose.count != 0)
                break;
            ReRender=true;
            break;
        case ConfigureNotify:
            /* call resizeGL only if our window-size changed */
            if ((event.xconfigure.width != width) ||
                    (event.xconfigure.height != height))
            {
                int width = event.xconfigure.width;
                int height = event.xconfigure.height;
                resizeGL(width, height);

            }
            ReRender=true;
            break;
        case KeyPress:
        {
            KeySym k=XLookupKeysym(&event.xkey, 0);

            if(key)
                *key=k;
            if(k>=XK_0 && k<=XK_9){

                int view_id=k-XK_0;

                char str[128];
                snprintf(str,128,"gl_view_%d",view_id);
                if(event.xkey.state&ShiftMask){
                    SaveMatrixFile(str);
                }else{
                    LoadMatrixFile(str);
                }

            }else{


                switch (k)
                {
                case XK_Escape:


                    cam->Reset(0,0,0,0,0,-1.0,0,1.0,0);
                    resetView();
                    break;
                case XK_a:
                    translateView(0.1,0,0);
                    break;
                case XK_d:
                    translateView(-0.1,0,0);
                    break;
                case XK_w:
                    translateView(0,0,-0.1);
                    break;
                case XK_s:
                    translateView(0,0,0.1);
                    break;
                case XK_r:
                    translateView(0,0.1,0);
                    break;
                case XK_f:
                    translateView(0,-0.1,0);
                    break;
                case XK_Left:
                    rotateViewY(5.0);
                    break;
                case XK_Right:
                    rotateViewY(-5.0);
                    break;
                case XK_Up:
                    rotateViewX(-5.0);
                    break;
                case XK_Down:
                    rotateViewX(5.0);
                    break;

                case XK_l:

                    //LoadMatrixFile("gl_matrix_temp");

                    break;

                case XK_p:
                    //SaveMatrixFile("gl_matrix_temp");
                    break;
                case XK_m:
                    RenderSurface=(RenderSurface+1)%3;
                    break;
                case XK_c:
                    RenderCuad=(RenderCuad+1)%3;
                    break;
                case XK_n:
                    RenderLines=(RenderLines+1)%3;
                    break;
                case XK_t:
                    ColorZmax+=0.2;
                    printf("\nGLViewer: Color Scale Max = %f\n",ColorZmax);
                    break;
                case XK_g:
                    ColorZmax-=0.2;
                    printf("\nGLViewer: Color Scale Max = %f\n",ColorZmax);
                    break;
                case XK_y:
                    ColorZmin+=0.2;
                    printf("\nGLViewer: Color Scale Min = %f\n",ColorZmin);
                    break;
                case XK_h:
                    ColorZmin-=0.2;
                    printf("\nGLViewer: Color Scale Min = %f\n",ColorZmin);
                    break;
                case XK_i:
                    RenderSigma=!RenderSigma;
                    break;
                case XK_v:
                    RenderTray=!RenderTray;
                    break;
                case XK_comma:
                    FixView=(FixView+1)%3;
                    printf("\nFixView: %d\n",FixView);
                    break;
                case XK_b:
                    rp.pos_tray->clear();
                    break;

                case XK_End:
                    return false;
                }
            }

            ReRender=true;

            break;
        }
        default:
            break;

        }
    }

    if(ReRender){
        if(!current)
            glXMakeCurrent(display, window, context);
        renderGL(rp);
    }

    return true;

}





void gl_viewer::SaveMatrixFile(const char *name){

    cam->Save(name);

    printf("\nGlview: %s Saved\n",name);


}


void gl_viewer::LoadMatrixFile(const char *name){


    cam->Load(name);

    printf("\nGlview: %s Loaded\n",name);

    resetView();



}
}
