/**
 * @file main.cpp
 * @brief fichero principal del proyecto
 * @author Juan Carlos Gámez
 * @version 1.0
 * @date junio 2012
 * @note Paso del la base de datos Biwi (www.vision.ee.ethz.ch/gfanelli) a fichero OpenNI
*/

// OpenCV includes.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

// OpenMP includes.
#include <omp.h>

// C++ includes.
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

//includes de mis clases
#include "MyKinect.h"

////includes para KINECT (OpenNI)
//#include <XnCppWrapper.h>
//#include <XnLog.h>
//
////includes para KINECT (freenect)
//#include "libKinect/libfreenect.h"
//#include "libKinect/libfreenect_cv.h"
//#include "libKinect/libfreenect_sync.h"


using namespace cv;
using namespace xn;
using namespace std;

// Indica el grado de depuración, el nivel al que se van a presentar los mensajes en pantalla
int debug=0;

/* ...................... Funciones ...................................*/

string writeSyntax(const char *applicationName){

    string sintaxis;

    sintaxis.assign("Sintaxis: ");
    sintaxis= sintaxis + applicationName + " ficheroXML directorioConFicherosBiwiRGByBin numMayorFrame ficheroOpenNI.oni fileRGB.avi fileDepth.avi \n";
    sintaxis= sintaxis + "\nNomenclatura de los ficheros: frame_nnnnn_rgb.png (fichero rgb) frame_nnnnn_depth.bin (fichero profundidad) \n";
    sintaxis= sintaxis + "\n donde los 'n' indican los números que van de 00001 a 'numMayorFrame' \n";
    sintaxis= sintaxis + "\nSalida: nombre de los ficheros correspondientes a los frames no encontrados \n";

    sintaxis= sintaxis + "\n Ejemplo: " + applicationName + " SampleConfig.xml /mnt/kinect_head_pose_db/01/ 501 ../01.oni ../01RGB.avi ../01Depth.avi\n\n";

    return sintaxis;
}

/* ................................................................*/

int checkSyntax(int argc, const char *argv[], string *inputXML, string *directorio,
                int *numFrameMaximo, string *ficheroOpenNI, string *videoOutRGB, string *videoOutDepth,
                string sintaxis){

    if (argc != 7){
        cout << "\n Número de parámetros incorrecto: " << argc << "\n " << endl;
        cout << sintaxis;
        return -1;
    }

    (*inputXML).assign(argv[1]);
    (*directorio).assign(argv[2]);
    (*numFrameMaximo)= atoi(argv[3]);
    (*ficheroOpenNI).assign(argv[4]);
    (*videoOutRGB).assign(argv[5]);
    (*videoOutDepth).assign(argv[6]);

    cout << endl;

    return 1;

}

/* ................................................................*/

int checkFiles(string inputXML, string directorio, string ficheroOpenNI, string videoOutRGB, string videoOutDepth){

    // comprobación de ficheros
    ifstream inputXMLStream;
    if (!inputXML.empty()){
        inputXMLStream.open(inputXML.c_str(), ios::in);
        if (inputXMLStream.is_open()){
            inputXMLStream.close();
        }
        else{
            cout << "ERROR: al abrir el fichero " << inputXML << endl;
            return -1;
        }
    }
    ifstream directorioStream;
    if (!directorio.empty()){
        directorioStream.open(directorio.c_str(), ios::in);
        if (directorioStream.is_open()){
            directorioStream.close();
        }
        else{
            cout << "ERROR: al acceder al directorio " << directorio << endl;
            return -1;
        }
    }

    ofstream ficheroOpenNIStream;
    if (!ficheroOpenNI.empty()){
        ficheroOpenNIStream.open(ficheroOpenNI.c_str(), ios::out);
        if (ficheroOpenNIStream.is_open()){
            ficheroOpenNIStream.close();
        }
        else{
            cout << "ERROR: al abrir el fichero " << ficheroOpenNI << endl;
            return -1;
        }
    }
    ofstream videoOutRGBStream;
    if (!videoOutRGB.empty()){
        videoOutRGBStream.open(videoOutRGB.c_str(), ios::out);
        if (videoOutRGBStream.is_open()){
            videoOutRGBStream.close();
        }
        else{
            cout << "ERROR: al abrir el fichero " << videoOutRGB << endl;
            return -1;
        }
    }
    ofstream videoOutDepthStream;
    if (!videoOutDepth.empty()){
        videoOutDepthStream.open(videoOutDepth.c_str(), ios::out);
        if (videoOutDepthStream.is_open()){
            videoOutDepthStream.close();
        }
        else{
            cout << "ERROR: al abrir el fichero " << videoOutDepth << endl;
            return -1;
        }
    }
    // FIN comprobación de ficheros
}
/* ................................................................*/

int checkFrame(string ficheroActualRGB, string ficheroActualDepth){

    // comprobación de ficheros
    ifstream ficheroActualRGBStream;
    if (!ficheroActualRGB.empty()){
        ficheroActualRGBStream.open(ficheroActualRGB.c_str(), ios::in);
        if (ficheroActualRGBStream.is_open()){
            ficheroActualRGBStream.close();
        }
        else{
//            cout << "ERROR: al abrir el fichero " << ficheroActualRGB << endl;
            return -1;
        }
    }
    ifstream ficheroActualDepthStream;
    if (!ficheroActualDepth.empty()){
        ficheroActualDepthStream.open(ficheroActualDepth.c_str(), ios::in);
        if (ficheroActualDepthStream.is_open()){
            ficheroActualDepthStream.close();
        }
        else{
//            cout << "ERROR: al abrir el fichero " << ficheroActualDepth << endl;
            return -1;
        }
    }
    // FIN comprobación de ficheros
}
/* ................................................................*/

int getKinect(MyImage *imgMyImage, string inputXML, MyKinect *kinect1){


    if (kinect1->init(inputXML) < 0){
       cout << "ERROR al iniciar la kinect con el fichero " << inputXML << endl;
       return -1;
    }

    if (kinect1->initGenerators() < 0){
        cout << "ERROR al iniciar los generadores de la Kinect" << endl;
        return -1;
    }


    // se leen dos frames para poder controlar cuando finaliza el fichero ONI
    if (kinect1->readFrame(imgMyImage) < 0){
        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
        return -1;
    }
    if (kinect1->readFrame(imgMyImage) < 0){
        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
        return -1;
    }

    return 1;
}

/* ................................................................*/

int openFiles(MyKinect *kinect1, int framesPerSeconds, string ficheroOpenNI,
              string videoOutRGB,VideoWriter *vidWritRGB,
              string videoOutDepth,VideoWriter *vidWritDepth){


    if (!videoOutRGB.empty()){
        (*vidWritRGB).open(videoOutRGB,CV_FOURCC('D','I','V','X'),framesPerSeconds,
                             Size(kinect1->getg_ImageMD()->XRes(),kinect1->getg_ImageMD()->YRes()) );
    }
    if (!videoOutDepth.empty()){
        (*vidWritDepth).open(videoOutDepth,CV_FOURCC('D','I','V','X'),framesPerSeconds,
                             Size(kinect1->getg_ImageMD()->XRes(),kinect1->getg_ImageMD()->YRes()), false );
    }

    //inicializar la captura y grabación a fichero .oni
    if (!ficheroOpenNI.empty()){
        kinect1->saveMockONIFile(ficheroOpenNI);
    }
    //FIN inicializar la captura y grabación a fichero .oni

    return 1;
}
/* ................................................................*/

int16_t* loadDepthImageCompressed( const char* fname ){

	//now read the depth image
	FILE* pFile = fopen(fname, "rb");
	if(!pFile){
		std::cerr << "could not open file " << fname << std::endl;
		return NULL;
	}

	int im_width = 0;
	int im_height = 0;
	bool success = true;

	success &= ( fread(&im_width,sizeof(int),1,pFile) == 1 ); // read width of depthmap
	success &= ( fread(&im_height,sizeof(int),1,pFile) == 1 ); // read height of depthmap

	int16_t* depth_img = new int16_t[im_width*im_height];

	int numempty;
	int numfull;
	int p = 0;

	while(p < im_width*im_height ){

		success &= ( fread( &numempty,sizeof(int),1,pFile) == 1 );

		for(int i = 0; i < numempty; i++)
			depth_img[ p + i ] = 0;

		success &= ( fread( &numfull,sizeof(int), 1, pFile) == 1 );
		success &= ( fread( &depth_img[ p + numempty ], sizeof(int16_t), numfull, pFile) == (unsigned int) numfull );
		p += numempty+numfull;

	}

	fclose(pFile);

	if(success)
		return depth_img;
	else{
		delete [] depth_img;
		return NULL;
	}
}

/* ................................................................*/

/* ................................................................*/

/* ................................................................*/

/* ................................................................*/

/* ................................................................*/



/* -------------------------- MAIN --------------------------------*/

int main( int argc, const char** argv )
{

    String inputXML;
    String directorio;
    String ficheroOpenNI;
    String videoOutRGB, videoOutDepth;
    String ficheroActualRGB, ficheroActualDepth;
    int i;

    // para la kinect
    MyKinect *kinect1;

    VideoWriter vidWritRGB, vidWritDepth;

    Mat matFrameRGB, matFrameDepth;
    MyImage *imgMyImage;

    int frames=1;
    int numFrameMaximo;

    int framesPerSeconds=30;

    // para medir tiempos
    clock_t t_ini, t_fin;
    double secs;

    string sintaxis;

    imgMyImage= new MyImage();
    imgMyImage->debug= debug;
    kinect1= new MyKinect();
    kinect1->debug= debug;

    // write the sintaxis variable
    sintaxis= writeSyntax(argv[0]);

    // check the application syntax and initialize variables
    if (checkSyntax(argc, argv, &inputXML, &directorio, &numFrameMaximo,
                    &ficheroOpenNI, &videoOutRGB, &videoOutDepth, sintaxis) == -1){
        return -1;
    }

    // check the files of the application
    if (checkFiles(inputXML, directorio, ficheroOpenNI, videoOutRGB, videoOutDepth) == -1){
        return -1;
    }

    // open .oni file and check the communication
    if (getKinect(imgMyImage, inputXML, kinect1) == -1){
        return -1;
    }

    // se leen dos frames para poder controlar el tamaño de las imagenes
    if (kinect1->readFrame(imgMyImage) < 0){
        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
        return -1;
    }

    // open files to write outputs such as videos, params and .oni file
    if (openFiles(kinect1, framesPerSeconds, ficheroOpenNI, videoOutRGB, &vidWritRGB,
                  videoOutDepth, &vidWritDepth) == -1){
        return -1;
    }

    // PROCESAMIENTO DE LOS FRAMES Y CREACIÓN DEL FICHERO .ONI
    while (frames <= numFrameMaximo){
        // preparación de los nombres de los ficheros
        char aux[10];
        sprintf(aux,"%05d", frames);
        ficheroActualRGB= directorio + "frame_" + String(aux) + "_rgb.png";
        ficheroActualDepth= directorio + "frame_" + String(aux) + "_depth.bin";

        // comprobación de la existencia del frame que vamos a tratar
        if (checkFrame(ficheroActualRGB, ficheroActualDepth) != -1){

            // procesar los ficheros RGB y Depth para guardarlos en el fichero .oni
            matFrameRGB = imread(ficheroActualRGB, 1);
            imgMyImage->setImg(matFrameRGB);
            //process depth data
            // Take current depth map
            int16_t* auxInt16= new int16_t[matFrameRGB.rows*matFrameRGB.cols];
            auxInt16= loadDepthImageCompressed(ficheroActualDepth.c_str());
            IplImage *iplDepthFrame= cvCreateImageHeader(cvSize(matFrameRGB.cols,matFrameRGB.rows),16,1);
            cvSetData(iplDepthFrame,auxInt16, 640*2);
            imgMyImage->setDepth(iplDepthFrame);
            cvReleaseImageHeader(&iplDepthFrame);
            delete auxInt16;

            kinect1->writeFrame(imgMyImage);

            // para comprobar la visualización
            imshow("ImgMyImage", imgMyImage->getImg());
            imshow("ImgMyImage->getShowDepth(0,g_Depth.GetDeviceMaxDepth()", imgMyImage->getShowDepth(0,kinect1->getg_Depth().GetDeviceMaxDepth()));
            char c = waitKey(20);
            switch( (char) c )
            {
            case 'p':
                waitKey();
                break;
            case 27:
                frames = numFrameMaximo + 1;
                break;
            }
            // FIN para comprobar la visualización

            // grabación a fichero .oni
            kinect1->record();
            //FIN grabación a fichero .oni

            // creación del vídeo original
            vidWritRGB << imgMyImage->getImg();   //creación del vídeo
            vidWritDepth << imgMyImage->getShowDepth(0,kinect1->getg_Depth().GetDeviceMaxDepth());

            t_fin= clock();
            secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
            (debug > 0) ? cout << "Tiempo de grabación: " <<  secs * 1000.0 << " milisegundos; nº de frames perdidos: " << secs * 1000.0 / 30.0 << endl : cout << "" ;

        }
        else{
            cout << "frame " << frames << " no encontrado" << endl;
        }
        frames++;
    }

    //finalizar la captura y grabación a fichero .oni
    kinect1->stopRecord();
    //FIN finalizar la captura y grabación a fichero .oni


    delete kinect1;

    return 0;
}


