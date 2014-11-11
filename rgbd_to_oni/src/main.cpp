
// // /**
// //  * @file main.cpp
// //  * @brief fichero principal del proyecto
// //  * @author Juan Carlos Gámez
// //  * @version 1.0
// //  * @date junio 2012
// //  * @note Paso del la base de datos Biwi (www.vision.ee.ethz.ch/gfanelli) a fichero OpenNI
// // */

// // // OpenCV includes.

// #include "cv.h"
// #include "highgui.h"
// #include "cvaux.h"
// #include <ros/ros.h>
// // // OpenMP includes.
// #include <omp.h>
// #include <boost/filesystem.hpp>
// #include <pcl/io/pcd_io.h>
// // C++ includes.
// #include <iostream>
// #include <fstream>
// #include <cstdio>
// #include <string>

//// //includes de mis clases
// #include "MyKinect.h"

// ////includes para KINECT (OpenNI)
// //#include <XnCppWrapper.h>
// //#include <XnLog.h>
// //
// ////includes para KINECT (freenect)
// //#include "libKinect/libfreenect.h"
// //#include "libKinect/libfreenect_cv.h"
// //#include "libKinect/libfreenect_sync.h"

// #define CHECK_RC(rc, what, flag)\
//     flag= 1;\
//     if (rc != XN_STATUS_OK)\
// {\
//     cout << what << "failed :"<< xnGetStatusString(rc);\
//     flag = 0;\
//     }\
//     cout << what << "succeed = " << flag;

// using namespace cv;
// using namespace xn;
// using namespace std;

// // // Indica el grado de depuración, el nivel al que se van a presentar los mensajes en pantalla
// // int debug=0;

// // /* ...................... Funciones ...................................*/

// // string writeSyntax(const char *applicationName){

// //     string sintaxis;

// //     sintaxis.assign("Sintaxis: ");
// //     sintaxis= sintaxis + applicationName + " ficheroXML directorioConFicherosBiwiRGByBin numMayorFrame ficheroOpenNI.oni fileRGB.avi fileDepth.avi \n";
// //     sintaxis= sintaxis + "\nNomenclatura de los ficheros: frame_nnnnn_rgb.png (fichero rgb) frame_nnnnn_depth.bin (fichero profundidad) \n";
// //     sintaxis= sintaxis + "\n donde los 'n' indican los números que van de 00001 a 'numMayorFrame' \n";
// //     sintaxis= sintaxis + "\nSalida: nombre de los ficheros correspondientes a los frames no encontrados \n";

// //     sintaxis= sintaxis + "\n Ejemplo: " + applicationName + " SampleConfig.xml /mnt/kinect_head_pose_db/01/ 501 ../01.oni ../01RGB.avi ../01Depth.avi\n\n";

// //     return sintaxis;
// // }

// // /* ................................................................*/

// // int checkSyntax(int argc, const char *argv[], string *inputXML, string *directorio,
// //                 int *numFrameMaximo, string *ficheroOpenNI, string *videoOutRGB, string *videoOutDepth,
// //                 string sintaxis){

// //     if (argc != 7){
// //         cout << "\n Número de parámetros incorrecto: " << argc << "\n " << endl;
// //         cout << sintaxis;
// //         return -1;
// //     }

// //     (*inputXML).assign(argv[1]);
// //     (*directorio).assign(argv[2]);
// //     (*numFrameMaximo)= atoi(argv[3]);
// //     (*ficheroOpenNI).assign(argv[4]);
// //     (*videoOutRGB).assign(argv[5]);
// //     (*videoOutDepth).assign(argv[6]);

// //     cout << endl;

// //     return 1;

// // }

// // /* ................................................................*/

// // int checkFiles(string inputXML, string directorio, string ficheroOpenNI, string videoOutRGB, string videoOutDepth){

// //     // comprobación de ficheros
// //     ifstream inputXMLStream;
// //     if (!inputXML.empty()){
// //         inputXMLStream.open(inputXML.c_str(), ios::in);
// //         if (inputXMLStream.is_open()){
// //             inputXMLStream.close();
// //         }
// //         else{
// //             cout << "ERROR: al abrir el fichero " << inputXML << endl;
// //             return -1;
// //         }
// //     }
// //     ifstream directorioStream;
// //     if (!directorio.empty()){
// //         directorioStream.open(directorio.c_str(), ios::in);
// //         if (directorioStream.is_open()){
// //             directorioStream.close();
// //         }
// //         else{
// //             cout << "ERROR: al acceder al directorio " << directorio << endl;
// //             return -1;
// //         }
// //     }

// //     ofstream ficheroOpenNIStream;
// //     if (!ficheroOpenNI.empty()){
// //         ficheroOpenNIStream.open(ficheroOpenNI.c_str(), ios::out);
// //         if (ficheroOpenNIStream.is_open()){
// //             ficheroOpenNIStream.close();
// //         }
// //         else{
// //             cout << "ERROR: al abrir el fichero " << ficheroOpenNI << endl;
// //             return -1;
// //         }
// //     }
// //     ofstream videoOutRGBStream;
// //     if (!videoOutRGB.empty()){
// //         videoOutRGBStream.open(videoOutRGB.c_str(), ios::out);
// //         if (videoOutRGBStream.is_open()){
// //             videoOutRGBStream.close();
// //         }
// //         else{
// //             cout << "ERROR: al abrir el fichero " << videoOutRGB << endl;
// //             return -1;
// //         }
// //     }
// //     ofstream videoOutDepthStream;
// //     if (!videoOutDepth.empty()){
// //         videoOutDepthStream.open(videoOutDepth.c_str(), ios::out);
// //         if (videoOutDepthStream.is_open()){
// //             videoOutDepthStream.close();
// //         }
// //         else{
// //             cout << "ERROR: al abrir el fichero " << videoOutDepth << endl;
// //             return -1;
// //         }
// //     }
// //     // FIN comprobación de ficheros
// // }
// // /* ................................................................*/

// // int checkFrame(string ficheroActualRGB, string ficheroActualDepth){

// //     cout<<ficheroActualDepth<<" depth path "<<endl;
// //     cout<<ficheroActualRGB<<" rbg path "<<endl;
// //     // comprobación de ficheros
// //     ifstream ficheroActualRGBStream;
// //     if (!ficheroActualRGB.empty()){
// //         ficheroActualRGBStream.open(ficheroActualRGB.c_str(), ios::in);
// //         if (ficheroActualRGBStream.is_open()){
// //             ficheroActualRGBStream.close();
// //         }
// //         else{
// //             cout << " RGB ERROR: al abrir el fichero " << ficheroActualRGB << endl;
// //             return -1;
// //         }
// //     }
// //     ifstream ficheroActualDepthStream;
// //     if (!ficheroActualDepth.empty()){
// //         ficheroActualDepthStream.open(ficheroActualDepth.c_str(), ios::in);
// //         if (ficheroActualDepthStream.is_open()){
// //             ficheroActualDepthStream.close();
// //         }
// //         else{
// //             cout << " Depth ERROR: al abrir el fichero " << ficheroActualDepth << endl;
// //             return -1;
// //         }
// //     }
// //     // FIN comprobación de ficheros
// // }
// // /* ................................................................*/

// // int getKinect(MyImage *imgMyImage, string inputXML, MyKinect *kinect1){

// //     kinect1->init();
// // //    if (kinect1->init(inputXML) < 0){
// // //       cout << "ERROR al iniciar la kinect con el fichero " << inputXML << endl;
// // //       return -1;
// // //    }

// // //    if (kinect1->initGenerators() < 0){
// // //        cout << "ERROR al iniciar los generadores de la Kinect" << endl;
// // //        return -1;
// // //    }


// // //    // se leen dos frames para poder controlar cuando finaliza el fichero ONI
// // //    if (kinect1->readFrame(imgMyImage) < 0){
// // //        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
// // //        return -1;
// // //    }
// // //    if (kinect1->readFrame(imgMyImage) < 0){
// // //        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
// // //        return -1;
// // //    }

// //     return 1;
// // }

// // /* ................................................................*/

// // int openFiles(MyKinect *kinect1, int framesPerSeconds, string ficheroOpenNI,
// //               string videoOutRGB,VideoWriter *vidWritRGB,
// //               string videoOutDepth,VideoWriter *vidWritDepth){


// //     if (!videoOutRGB.empty()){
// //         cout<<" 217 "<<endl;
// //         (*vidWritRGB).open(videoOutRGB,CV_FOURCC('D','I','V','X'),framesPerSeconds,
// //                              Size(kinect1->getg_ImageMD()->XRes(),kinect1->getg_ImageMD()->YRes()) );
// //         cout<<" 220 "<<endl;
// //     }
// //     if (!videoOutDepth.empty()){
// //         cout<<" 223 "<<endl;
// //         (*vidWritDepth).open(videoOutDepth,CV_FOURCC('D','I','V','X'),framesPerSeconds,
// //                              Size(kinect1->getg_ImageMD()->XRes(),kinect1->getg_ImageMD()->YRes()), false );
// //         cout<<" 226 "<<endl;
// //     }

// //     //inicializar la captura y grabación a fichero .oni
// //     if (!ficheroOpenNI.empty()){
// //         cout<<" 231 "<<endl;
// //         kinect1->saveMockONIFile(ficheroOpenNI);
// //         cout<<" 233 "<<endl;
// //     }
// //     //FIN inicializar la captura y grabación a fichero .oni

// //     return 1;
// // }
// // /* ................................................................*/

// // int16_t* loadDepthImageCompressed( const char* fname ){

// // 	//now read the depth image
// // 	FILE* pFile = fopen(fname, "rb");
// // 	if(!pFile){
// // 		std::cerr << "could not open file " << fname << std::endl;
// // 		return NULL;
// // 	}

// // 	int im_width = 0;
// // 	int im_height = 0;
// // 	bool success = true;

// // 	success &= ( fread(&im_width,sizeof(int),1,pFile) == 1 ); // read width of depthmap
// // 	success &= ( fread(&im_height,sizeof(int),1,pFile) == 1 ); // read height of depthmap

// // 	int16_t* depth_img = new int16_t[im_width*im_height];

// // 	int numempty;
// // 	int numfull;
// // 	int p = 0;

// // 	while(p < im_width*im_height ){

// // 		success &= ( fread( &numempty,sizeof(int),1,pFile) == 1 );

// // 		for(int i = 0; i < numempty; i++)
// // 			depth_img[ p + i ] = 0;

// // 		success &= ( fread( &numfull,sizeof(int), 1, pFile) == 1 );
// // 		success &= ( fread( &depth_img[ p + numempty ], sizeof(int16_t), numfull, pFile) == (unsigned int) numfull );
// // 		p += numempty+numfull;

// // 	}

// // 	fclose(pFile);

// // 	if(success)
// // 		return depth_img;
// // 	else{
// // 		delete [] depth_img;
// // 		return NULL;
// // 	}
// // }

// // /* ................................................................*/

// // /* ................................................................*/

// // /* ................................................................*/

// // /* ................................................................*/

// // /* ................................................................*/



// // /* -------------------------- MAIN --------------------------------*/

// // int main( int argc, const char** argv )
// // {

// //     String inputXML= "/home/shaghayegh/backup/SamplesConfig.xml";
// //     String directorio = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512164529/";
// //     String ficheroOpenNI = "/home/shaghayegh/up.oni";
// //     String videoOutRGB = "/home/shaghayegh/backup/rgb.avi" ;
// //     String videoOutDepth = "/home/shaghayegh/backup/depth.avi";
// //     String ficheroActualRGB, ficheroActualDepth;
// //     int i;

// //     // para la kinect
// //     MyKinect *kinect1;

// //     VideoWriter vidWritRGB, vidWritDepth;

// //     Mat matFrameRGB, matFrameDepth;
// //     MyImage *imgMyImage;

// //     int frames=1;
// //     int numFrameMaximo = 100;

// //     int framesPerSeconds=30;

// //     // para medir tiempos
// //     clock_t t_ini, t_fin;
// //     double secs;

// //     string sintaxis;

// //     imgMyImage= new MyImage();
// //     imgMyImage->debug= debug;
// //     kinect1= new MyKinect();
// //     kinect1->debug= debug;

// //     // write the sintaxis variable
// //     sintaxis= writeSyntax(argv[0]);
// //     cout<<" wow "<<endl;
// //     // check the application syntax and initialize variables
// // //    if (checkSyntax(argc, argv, &inputXML, &directorio, &numFrameMaximo,
// // //                    &ficheroOpenNI, &videoOutRGB, &videoOutDepth, sintaxis) == -1){
// // //        return -1;
// // //    }
// //     cout<<" 334 "<<endl;

// //     // check the files of the application
// //     if (checkFiles(inputXML, directorio, ficheroOpenNI, videoOutRGB, videoOutDepth) == -1){
// //         return -1;
// //     }
// //     cout<<" 340 "<<endl;

// //     // open .oni file and check the communication
// // //    if (getKinect(imgMyImage, inputXML, kinect1) == -1){
// // //        return -1;
// // //    }
// //     cout<<" 346 "<<endl;

// //     // se leen dos frames para poder controlar el tamaño de las imagenes
// // //    if (kinect1->readFrame(imgMyImage) < 0){//az kinect tasvir rgb va depth mikhune tabdil be mat mikone rgb be tasvir u8c3 depth ham be u16c1
// // //ba iplimage kar karde negah kon shayad to ham az in no dorost koni dorost she
// //     //        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
// // //        return -1;
// // //    }
// //     cout<<" 353 "<<endl;


// //     // open files to write outputs such as videos, params and .oni file
// //     if (openFiles(kinect1, framesPerSeconds, ficheroOpenNI, videoOutRGB, &vidWritRGB,
// //                   videoOutDepth, &vidWritDepth) == -1){
// //         return -1;
// //     }
// // cout<<" 356 "<<endl;
// //     // PROCESAMIENTO DE LOS FRAMES Y CREACIÓN DEL FICHERO .ONI
// //     while (frames <= numFrameMaximo){
// //         // preparación de los nombres de los ficheros
// //         char aux[10];
// //         sprintf(aux,"%05d", frames);
// //         ficheroActualRGB= directorio + "RGB_" + boost::to_string(frames) + ".png";
// //         ficheroActualDepth= directorio + "Depth_" + boost::to_string(frames) + ".png";
// // //        checking the existence of the frame we will try
// //         // comprobación de la existencia del frame que vamos a tratar
// //         if (checkFrame(ficheroActualRGB, ficheroActualDepth) != -1){
// //             //process the RGB and Depth to save files in the file
// //             // procesar los ficheros RGB y Depth para guardarlos en el fichero .oni

// //             matFrameRGB = imread(ficheroActualRGB, 1);
// //             imgMyImage->setImg(matFrameRGB);
// //             matFrameDepth = imread(ficheroActualDepth,1);
// //             imgMyImage->setDepth(matFrameDepth);
// //             //process depth data
// //             // Take current depth map
// // //?            int16_t* auxInt16= new int16_t[matFrameRGB.rows*matFrameRGB.cols];
// // // ?           auxInt16= loadDepthImageCompressed(ficheroActualDepth.c_str());
// // //  ?          IplImage *iplDepthFrame= cvCreateImageHeader(cvSize(matFrameRGB.cols,matFrameRGB.rows),16,1);
// // //   ?         cvSetData(iplDepthFrame,auxInt16, 640*2);
// // // ?           imgMyImage->setDepth(iplDepthFrame);

// // //?            cvReleaseImageHeader(&iplDepthFrame);
// // // ?           delete auxInt16;
// //             cout<<" 390 "<<endl;
// //             kinect1->writeFrame(imgMyImage);
// //             cout<<" 392 "<<endl;
// //             // para comprobar la visualización
// //             imshow("ImgMyImage", imgMyImage->getImg());
// //                        cout<<" 395 "<<endl;
// //             imshow("ImgMyImage->getShowDepth(0,g_Depth.GetDeviceMaxDepth()", imgMyImage->getDepth());
// //             //imgMyImage->getShowDepth(0,kinect1->getg_Depth().GetDeviceMaxDepth())
// //             cout<<" 396 "<<endl;
// //             char c = waitKey(20);
// //             switch( (char) c )
// //             {
// //             case 'p':
// //                 waitKey();
// //                 break;
// //             case 27:
// //                 frames = numFrameMaximo + 1;
// //                 break;
// //             }
// //             // FIN para comprobar la visualización

// //             // grabación a fichero .oni
// //             kinect1->record();
// //             //FIN grabación a fichero .oni
// //             cout<<" 414 "<<endl;

// //             // creación del vídeo original
// //             vidWritRGB << imgMyImage->getImg();   //creación del vídeo
// //             vidWritDepth << imgMyImage->getShowDepth(0,kinect1->getg_Depth().GetDeviceMaxDepth());

// //             t_fin= clock();
// //             secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
// //             (debug > 0) ? cout << "Tiempo de grabación: " <<  secs * 1000.0 << " milisegundos; nº de frames perdidos: " << secs * 1000.0 / 30.0 << endl : cout << "" ;

// //         }
// //         else{
// //             cout << "frame " << frames << " no encontrado" << endl;
// //         }
// //         frames++;
// //     }

// //     //finalizar la captura y grabación a fichero .oni
// //     kinect1->stopRecord();
// //     //FIN finalizar la captura y grabación a fichero .oni


// //     delete kinect1;

// //     return 0;
// // }

// String DepthFile,RGBFile;
// xn::Context context;
// XnStatus nRetVal;
// xn::Player Player_man;
// //Generators
// bool context_init;
// xn::DepthGenerator depth;   bool depth_init;
// xn::ImageGenerator color;   bool color_init;
// xn::UserGenerator user;     bool user_init;
// xn::Recorder recorder;
// bool user_cloud_populated;
// // Data types
// xn::DepthMetaData depthMD;  //Deth Meta data
// xn::ImageMetaData imageMD;  //Image Meta Data
// xn::SceneMetaData sceneMD;  //Scene meta data - It will be used to segment the USER
// const XnRGB24Pixel* pImageMap;    //Pointer to color map
// const XnDepthPixel* pDepthMap;    //Pointer to Depth map
// bool recorder_init;
// bool recorder_file,recorder_depth, recorder_color;
// bool startgeneration_init;
// bool aligment;
// MockDepthGenerator mockg_Depth;
// MockImageGenerator mockg_Image;

// xn::MockRawGenerator rawGenerator; bool rawGenerator_init;

// int writeFrame(MyImage *imgMyImage);
// int main( int argc,  char** argv )
// {
//     ros::init(argc, argv, "openni_tracker");
//     String directorio = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512164529/";
//         nRetVal = XN_STATUS_OK;
//     depth_init =0;
//     color_init =0;
//     context_init =0;
//     recorder_init =0;
//     MyImage *imgMyImage = new MyImage;

//     context.Release();//@@
//     nRetVal = context.Init();//@@?initializ kardane kinect
//     CHECK_RC(nRetVal, "Intialization",context_init);//@@

//         nRetVal = context.OpenFileRecording( "/home/shaghayegh/backup/up.oni", Player_man);//@@?baz kardane fili ke tush etelaat zakhire shode
//     CHECK_RC(nRetVal, "Recording opening",context_init);//@@
//     // Intialize Depth
//     nRetVal = depth.Create(context);
//     CHECK_RC(nRetVal, "Depth Initialization",depth_init);


//     // Intialize Color
//     nRetVal = color.Create(context);
//     CHECK_RC(nRetVal, "Color Initialization",color_init);

//     // Intialize User
//     nRetVal = user.Create(context);
//     CHECK_RC(nRetVal, "Color Initialization",user_init);



//     if(depth.IsCapabilitySupported("AlternativeViewPoint"))
//     {
//         nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(color);
//         CHECK_RC(nRetVal,"Depth and Color Aligment",aligment);
//     }
//     else
//     {
//         cout<< "Depth and Color aligment is  **** NOT **** possible";
//     }

//     // Creating Production Tree
//     context.CreateAnyProductionTree(XN_NODE_TYPE_RECORDER,NULL,recorder)   ;
//     CHECK_RC(nRetVal, "Create recorder",recorder_init);


//     nRetVal = recorder.SetDestination(XN_RECORD_MEDIUM_FILE, "/home/shaghayegh/up.oni" );
//     CHECK_RC(nRetVal, "Setting up recorder file ",recorder_file);

//     nRetVal = recorder.AddNodeToRecording(depth, XN_CODEC_16Z_EMB_TABLES);
//     CHECK_RC(nRetVal, "Adding Node to Recorder - Depth ",recorder_depth);

//     nRetVal = recorder.AddNodeToRecording(color);
//     CHECK_RC(nRetVal, "Adding Node to Recorder - Color ",recorder_color);

//     nRetVal = context.StartGeneratingAll();
// //    CHECK_RC(nRetVal, "Kinect Final configuration ",startgeneration_init);
//     // boost::thread Update_Process(&UpdateContext);
//     // ros::Rate r(30);
// //    ros::NodeHandle pnh("~");
// //    string frame_id("openni_depth_frame");
// //    pnh.getParam("camera_frame_id", frame_id);
//     int frames = 0;
//     while (ros::ok()) {
//               nRetVal = context.WaitAnyUpdateAll(); // es el que va mejor!!!!!
////         if(frames <1000)
////         {
////             ++frames;
////             RGBFile= directorio + "RGB_" + boost::to_string(frames) + ".png";
////             DepthFile= directorio + "Depth_" + boost::to_string(frames) + ".png";
////             // //        checking the existence of the frame we will try
////             Mat matFrameRGB = imread(RGBFile, 1);
////             imgMyImage->setImg(matFrameRGB);
////             Mat matFrameDepth = imread(DepthFile,1);
////             imgMyImage->setDepth(matFrameDepth);
////             writeFrame(imgMyImage);

//             recorder.Record();
//             ros::spinOnce();
////         }
////         else
////         {
////             ros::shutdown();
////         }
//     }
//     if(depth_init) depth.Release();
//     if(color_init) color.Release();
//     if(context_init) context.Release();
//     if(recorder_init) recorder.Release();

//     context.Shutdown();
//     return 0;
// }
// int writeFrame(MyImage *imgMyImage){

//     XnStatus rc = XN_STATUS_OK;
//     EnumerationErrors errors;

// //    rc = context.WaitAnyUpdateAll(); // es el que va mejor!!!!!

// //    if (rc != XN_STATUS_OK){
// //        printf("%s\n", xnGetStatusString(rc));
// //        return(-1);
// //    }
//     if (color.IsValid()){
//         //cout <<  "IMG VALIDO" << endl;
//         color.GetMetaData(imageMD);
// //        cout<<color.GetMetaData(<<"   sssss"<<endl;
//     }
//     if (depth.IsValid()){
//         //cout <<  "DEPTH VALIDO" << endl;
//         depth.GetMetaData(depthMD);
//     }

//     nRetVal= imageMD.MakeDataWritable();
//     if (nRetVal != XN_STATUS_OK){
//         printf("g_ImageMD.MakeDataWritable(); - %s\n", xnGetStatusString(nRetVal));
//         return(-1);
//     }

//     // Take current image
//     RGB24Map& imageMap = imageMD.WritableRGB24Map();
//     cout<<imageMap.YRes()<<" salam "<<endl;
//     cout<<imageMap(0,0).nBlue<<" hey "<<endl;
//     for (XnUInt32 y = 0; y < imageMap.YRes(); y++)
//     {
//         for (XnUInt32 x = 0; x < imageMap.XRes(); x++)
//         {
//             XnRGB24Pixel* imagePixel;

//             imagePixel->nBlue=imgMyImage->getValueB(x,y);
//             imagePixel->nGreen=imgMyImage->getValueG(x,y);
//             imagePixel->nRed=imgMyImage->getValueR(x,y);

//             imageMap(x,y) = *imagePixel;
//         }
//     }

//     //write data really"
// //    nRetVal= mockg_Image.SetData(imageMD);
// //    if (nRetVal != XN_STATUS_OK){
// //        printf("mockg_Image.SetData(g_ImageMD); - %s\n", xnGetStatusString(nRetVal));
// //        return(-1);
// //    }

//     //process depth data
//     // hacer que se pueda modificar
//     nRetVal= depthMD.MakeDataWritable();
//     if (nRetVal != XN_STATUS_OK){
//         printf("g_DepthMD.MakeDataWritable(); - %s\n", xnGetStatusString(nRetVal));
//         return(-1);
//     }


//     // Take current depth map
//     DepthMap& depthMap = depthMD.WritableDepthMap();
//     for (XnUInt32 y = 0; y < depthMap.YRes(); y++)
//     {
//         for (XnUInt32 x = 0; x < depthMap.XRes(); x++)
//         {
//             //Punch vertical cut lines in the depth image
//             depthMap(x,y) = imgMyImage->getValueDepth(x,y);
//         }
//     }

//     //escribir los datos "realmente"
// //    nRetVal= mockg_Depth.SetData(depthMD);
// //    if (nRetVal != XN_STATUS_OK){
// //        printf("mockg_Depth.SetData(g_DepthMD); - %s\n", xnGetStatusString(nRetVal));
// //        return(-1);
// //    }

//     //cout << "MyKinect::writeFrame" << endl;
//     return 1;
// }
// int MyKinect::saveMockONIFile(String outputName){

//     XnStatus rc= XN_STATUS_OK;
//     NodeInfoList recorderList;
//     rc= g_Context.EnumerateProductionTrees(XN_NODE_TYPE_RECORDER,NULL,recorderList);
//     if (rc != XN_STATUS_OK){
//         printf("ERROR creación de 'enumerate recorders' %s\n", xnGetStatusString(rc));
//         return(-1);
//     }
//     // coger el primero de la lista de "recorder"
//     NodeInfo chosen = *recorderList.Begin();

//     rc= g_Context.CreateProductionTree(chosen);
//     if (rc != XN_STATUS_OK){
//         printf("ERROR creacion de 'recorder' %s\n", xnGetStatusString(rc));
//         return(-1);
//     }

//     pRecorder= new Recorder();
//     rc= chosen.GetInstance(*(pRecorder));
//     if (rc != XN_STATUS_OK){
//         printf("ERROR creacion de 'get recorder instance' %s\n", xnGetStatusString(rc));
//         return(-1);
//     }

//     rc= pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, outputName.c_str());
//     if (rc != XN_STATUS_OK){
//         printf("ERROR creacion de fichero .oni (%s) %s\n", outputName.c_str(), xnGetStatusString(rc));
//         return(-1);
//     }

//     rc = mockg_Depth.CreateBasedOn(g_Depth);
//     if (rc != XN_STATUS_OK){
//         printf("ERROR crear mockg_Depth.CreateBasedOn(g_Depth) %s\n", xnGetStatusString(rc));
//         return(-1);
//     }
//     if (mockg_Depth.IsValid()){
//         (debug > 1) ? cout << "graba depth" << endl : cout << "";
//         rc= pRecorder->AddNodeToRecording(mockg_Depth, XN_CODEC_16Z_EMB_TABLES);
//         if (rc != XN_STATUS_OK){
//             printf("ERROR add depth node %s\n", xnGetStatusString(rc));
//             return(-1);
//         }
//     }

//     rc = mockg_Image.CreateBasedOn(g_Image);
//     if (rc != XN_STATUS_OK){
//         printf("ERROR crear mockg_Image.CreateBasedOn(g_Image) %s\n", xnGetStatusString(rc));
//         return(-1);
//     }
//     if (mockg_Image.IsValid()){
//         (debug > 1) ? cout << "graba image" << endl : cout << "";
//         rc= pRecorder->AddNodeToRecording(mockg_Image, XN_CODEC_JPEG);
//         if (rc != XN_STATUS_OK){
//             printf("ERROR add image node %s\n", xnGetStatusString(rc));
//             return(-1);
//         }
//     }

//     (debug > 1) ? cout << "MyKinect::saveMockONIFile" << endl : cout << "";
// }


/**
 * @file main.cpp
 * @brief fichero principal del proyecto
 * @author Juan Carlos Gámez
 * @version 1.0
 * @date junio 2012
 * @note Paso del la base de datos Biwi (www.vision.ee.ethz.ch/gfanelli) a fichero OpenNI
*/

#include <ros/ros.h>
// // OpenMP includes.
#include <omp.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
// C++ includes.
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

// OpenCV includes.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

// OpenMP includes.
// #include <omp.h>

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
//    	cout<<ficheroOpenNI<<" address "<<endl;
//    	kinect1->saveONIFile(ficheroOpenNI);
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


    String inputXML= "/home/shaghayegh/backup/SamplesConfig.xml";
    String directorio = "/media/6B58CB581C0AACF6/ebook/Articles/activity_recognition/data1/0512164529/";
    String ficheroOpenNI = "/home/shaghayegh/up.oni";
    String videoOutRGB = "/home/shaghayegh/backup/rgb.avi" ;
    String videoOutDepth = "/home/shaghayegh/backup/depth.avi";

    String ficheroActualRGB, ficheroActualDepth;
    int i;

    // para la kinect
    MyKinect *kinect1;

    VideoWriter vidWritRGB, vidWritDepth;

    Mat matFrameRGB, matFrameDepth;
    MyImage *imgMyImage;

    int frames=1;
    int numFrameMaximo = 300;

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
    // if (checkSyntax(argc, argv, &inputXML, &directorio, &numFrameMaximo,
    //                 &ficheroOpenNI, &videoOutRGB, &videoOutDepth, sintaxis) == -1){
    //     return -1;
    // }

    // check the files of the application
    if (checkFiles(inputXML, directorio, ficheroOpenNI, videoOutRGB, videoOutDepth) == -1){
        return -1;
    }
cout<<"1078"<<endl;

    // open .oni file and check the communication
    if (getKinect(imgMyImage, inputXML, kinect1) == -1){
        return -1;
    }
cout<<"1083"<<endl;

    // se leen dos frames para poder controlar el tamaño de las imagenes
    if (kinect1->readFrame(imgMyImage) < 0){
        cout << "ERROR al leer el frame " << kinect1->getg_ImageMD()->FrameID() << endl;
        return -1;
    }
cout<<"1089"<<endl;

    // open files to write outputs such as videos, params and .oni file
    if (openFiles(kinect1, framesPerSeconds, ficheroOpenNI, videoOutRGB, &vidWritRGB,
                  videoOutDepth, &vidWritDepth) == -1){
        return -1;
    }
//    // PROCESAMIENTO DE LOS FRAMES Y CREACIÓN DEL FICHERO .ONI
//    while (frames <= numFrameMaximo){
//        // preparación de los nombres de los ficheros
//        char aux[10];
//        sprintf(aux,"%05d", frames);
//        ficheroActualRGB= directorio +"RGB_" + boost::to_string(frames) + ".png";
//        ficheroActualDepth= directorio +"Depth_"+ boost::to_string(frames) + ".png";

//        // comprobación de la existencia del frame que vamos a tratar
//        if (checkFrame(ficheroActualRGB, ficheroActualDepth) != -1){

//            // procesar los ficheros RGB y Depth para guardarlos en el fichero .oni
//            matFrameRGB = imread(ficheroActualRGB, 1);
//            resize(matFrameRGB, matFrameRGB, Size(640, 480), 0, 0, INTER_CUBIC);
//            imgMyImage->setImg(matFrameRGB);
//            matFrameDepth = imread(ficheroActualDepth,1);
//            resize(matFrameDepth, matFrameDepth, Size(640, 480), 0, 0, INTER_CUBIC);
//             imgMyImage->setDepth(matFrameDepth);
//            //process depth data
//            // Take current depth map
////             int16_t* auxInt16= new int16_t[matFrameRGB.rows*matFrameRGB.cols];
////             auxInt16= loadDepthImageCompressed(ficheroActualDepth.c_str());
////             IplImage *iplDepthFrame= cvCreateImageHeader(cvSize(matFrameRGB.cols,matFrameRGB.rows),16,1);
////             cvSetData(iplDepthFrame,auxInt16, 320*2);
////             imgMyImage->setDepth(iplDepthFrame);
////             cvReleaseImageHeader(&iplDepthFrame);
////             delete auxInt16;

//            kinect1->writeFrame(imgMyImage);
//            // para comprobar la visualización
//            imshow("ImgMyImage", imgMyImage->getImg());
//            imshow("depth ",imgMyImage->getDepth());

//            char c = waitKey(20);
//            switch( (char) c )
//            {
//            case 'p':
//                waitKey();
//                break;
//            case 27:
//                frames = numFrameMaximo + 1;
//                break;
//            }


//            // FIN para comprobar la visualización

//            // grabación a fichero .oni
//            kinect1->record();
//            //FIN grabación a fichero .oni

//            // creación del vídeo original
//            vidWritRGB << imgMyImage->getImg();   //creación del vídeo
//            vidWritDepth << imgMyImage->getShowDepth(0,kinect1->getg_Depth().GetDeviceMaxDepth());

//            t_fin= clock();
//            secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
//            (debug > 0) ? cout << "Tiempo de grabación: " <<  secs * 1000.0 << " milisegundos; nº de frames perdidos: " << secs * 1000.0 / 30.0 << endl : cout << "" ;

//        }
//        else{
//            cout << "frame " << frames << " no encontrado" << endl;
//        }
//        frames++;
//    }

//    //finalizar la captura y grabación a fichero .oni
//    kinect1->stopRecord();
//    //FIN finalizar la captura y grabación a fichero .oni


    delete kinect1;

    return 0;
}


