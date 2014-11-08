/**
 * @file MyImage
 * @brief fichero de clase genérica de imágenes
 * @author Juan Carlos Gámez
 * @version 1.2
 * @date junio 2012
 * @note Fichero genérico de imágenes
*/

#ifndef MYIMAGE_H
#define MYIMAGE_H


// OpenCV includes.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"


//includes para KINECT (OpenNI)
#include <XnCppWrapper.h>
#include <XnLog.h>

#include <iostream>

using namespace cv;
using namespace xn;
using namespace std;

class MyImage
{
    public:
        /** Default constructor */
        MyImage();
        /** Default destructor */
        ~MyImage();
        /** Constructor */
        MyImage(Mat image);
        /** Default constructor */
        MyImage(IplImage image);
        /** Constructor de copia */
        MyImage(const MyImage &image);
        /** Operador de asignaci�n por copia */
        MyImage & operator=(const MyImage &image);
        /** Access img
         * \return The current value of img
         */
        Mat getImg();
        /** Set img
         * \param val New value to set
         */
        void setImg(Mat val);

        /** Access depth
         * \return The current value of img
         */
        Mat getDepth();
        /** Access depth tipo Mat
         * \param min valor mínimo a considerar
         * \param max valor máximo a considerar
         * \return The current value of img
         */
        Mat getShowDepth(int min, int max);
        /** Set depth
         * \param val New value to set
         */
        void setDepth(Mat val);
        /** Access imgGray
         * \return The current value of imgGray
         */
        Mat getImgGray();
        /** Set imgGray
         * \param val New value to set
         */
        void setImgGray(Mat val);
        /** Access img
         * \return The current value of img
         * @note Puntero a la imagen, OJO se puede modificar desde fuera
         */
        IplImage getIplImg();
        /** Set img
         * \param val New value to set
         */
        void setImg(IplImage val);

        /** Access imgGray
         * \return The current value of imgGray
         * @note Puntero a la imagen, OJO se puede modificar desde fuera
         */
        IplImage getIplImgGray();
        /** Set imgGray
         * \param val New value to set
         */
        void setImgGray(IplImage val);

        /** Access depth
         * \return The current value of imgGray
         * @note Puntero a la imagen, OJO se puede modificar desde fuera
         */
        IplImage getIplDepth();
        /** Set imgGray
         * \param val New value to set
         */
        void setDepth(IplImage val);

        /** Get width
         * @return image width
         */
         int getWidth();
         /** get Height
         * @return image height
         */
         int getHeight();

         /** get Value channel R
         * @param x pos X del pixel en el canal R
         * @param y pos Y del pixel en el canal R
         * @return int valor del pixel (x,y) del canal R
         */
         int getValueR(int x, int y);
         /** set Value channel R
         * @param x pos X del pixel en el canal R
         * @param y pos Y del pixel en el canal R
         * @param valor del pixel (x,y) del canal R
         */
         void setValueR(int x, int y, int v);
         /** get Value channel G
         * @param x pos X del pixel en el canal G
         * @param y pos Y del pixel en el canal G
         * @return int valor del pixel (x,y) del canal G
         */
         int getValueG(int x, int y);
         /** set Value channel G
         * @param x pos X del pixel en el canal G
         * @param y pos Y del pixel en el canal G
         * @param valor del pixel (x,y) del canal G
         */
         void setValueG(int x, int y, int v);
         /** get Value channel B
         * @param x pos X del pixel en el canal B
         * @param y pos Y del pixel en el canal B
         * @return int valor del pixel (x,y) del canal B
         */
         int getValueB(int x, int y);
         /** set Value channel B
         * @param x pos X del pixel en el canal B
         * @param y pos Y del pixel en el canal B
         * @param valor del pixel (x,y) del canal B
         */
         void setValueB(int x, int y, int v);
         /** get Value channel B
         * @param x pos X del pixel en el canal Gray
         * @param y pos Y del pixel en el canal Gray
         * @return int valor del pixel (x,y) del canal Gray
         */
         int getValueGray(int x, int y);
         /** set Value channel Gray
         * @param x pos X del pixel en el canal Gray
         * @param y pos Y del pixel en el canal Gray
         * @param valor del pixel (x,y) del canal Gray
         */
         void setValueGray(int x, int y, int v);

         /** get Value Depth
         * @param x pos X del pixel en depth
         * @param y pos Y del pixel en depth
         * @return int valor del pixel (x,y) de depth
         */
        int getValueDepth(int x, int y);
         /** set Value channel Depth
         * @param x pos X del pixel en depth
         * @param y pos Y del pixel en depth
         * @param valor del pixel (x,y) de depth
         */
        void setValueDepth(int x, int y, int v);
         /** get frame value of image
          * @return int frame value of image
         */
        int getFrameImg();
         /** set frame value of image
          * @param v int frame value of image
         */
        void setFrameImg(int v);
         /** get frame value of depth
          * @return int frame value of depth
         */
        int getFrameDepth();
         /** set frame value of depth
          * @param v int frame value of depth
         */
        void setFrameDepth(int v);

        // Indica el grado de depuración, el nivel al que se van a presentar los mensajes en pantalla
        int debug;

    protected:
        Mat img; //!< Member variable "img"
        Mat imgGray; //!< Member variable "imgGray"
        Mat depth;
        int frameImg;
        int frameDepth;

    private:

};

#endif // MYIMAGE_H
