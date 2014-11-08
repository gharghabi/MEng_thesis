#include "MyKinect.h"

using namespace xn;

/** Default constructor */
MyKinect::MyKinect()
{
    //ctor
    freenect_angle= 0;
    led= LED_BLINK_GREEN;
    numFramesImage=0;
    numFramesDepth=0;

    debug= 0;

//cout << "MyKinect constructor" << endl;
}

MyKinect::~MyKinect(){

    g_Context.Shutdown();

    player.Release();
    g_Depth.Release();
    g_Image.Release();
    g_User.Release();
    analyScene.Release();
    mockg_Depth.Release();
    mockg_Image.Release();
    g_Context.Release();

//cout << "MyKinect destructor" << endl;
}

/**
Inicializa la Kinect sin fichero XML (para lectura de fichero ONI)
*/
int MyKinect::init(){

    XnStatus rc= XN_STATUS_OK;
    rc= g_Context.Init();
    if (rc != XN_STATUS_OK){
        printf("ERROR en llamada a Init %s\n", xnGetStatusString(rc));
        return(-1);
    }
//cout << "MyKinect::init()" << endl;
    return 1;
}

/**
* Lectura de un fichero ONI como si estuviese trabajando con la kinect directamente
*/
int MyKinect::readONIFile(String inputName){

    XnStatus rc = XN_STATUS_OK;
    rc= g_Context.OpenFileRecording(inputName.c_str(),player);
    if (rc != XN_STATUS_OK){
        printf("ERROR lectura de fichero oni openFileRecording(%s) %s\n", inputName.c_str(), xnGetStatusString(rc));
        return(-1);
    }
//cout << "MyKinect::readONIFile" << endl;
    return 1;
}


/**
Inicializa la Kinect con fichero XML (para captura desde kinect directamente)
*/
int MyKinect::init(String arg_fileXML){

    fileXML.assign(arg_fileXML);

    XnStatus rc = XN_STATUS_OK;
    EnumerationErrors errors;
    rc = g_Context.InitFromXmlFile(fileXML.c_str(), &errors);
    if (rc == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        g_Context.Shutdown();
        return(-1);
    }
    else if (rc != XN_STATUS_OK)
    {
        printf("Open failed: %s\n", xnGetStatusString(rc));
        g_Context.Shutdown();
        return(-1);
    }
//cout << "MyKinect::init(fileXML)" << endl;
    return 1;
}

/**
Inicializa los Generadores de la Kinect
*/
int MyKinect::initGenerators()
{
	XnStatus rc = XN_STATUS_OK;

	NodeInfoList list;
	rc = g_Context.EnumerateExistingNodes(list);
	if (rc == XN_STATUS_OK)
	{
		for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it)
		{
			switch ((*it).GetDescription().Type)
			{
			case XN_NODE_TYPE_DEVICE:
				(*it).GetInstance(g_Device);
				break;
			case XN_NODE_TYPE_DEPTH:
				(*it).GetInstance(g_Depth);
				break;
			case XN_NODE_TYPE_IMAGE:
				(*it).GetInstance(g_Image);
				break;
			case XN_NODE_TYPE_IR:
				(*it).GetInstance(g_IR);
				break;
			case XN_NODE_TYPE_USER:
				(*it).GetInstance(g_User);
				break;
			case XN_NODE_TYPE_SCENE:
				(*it).GetInstance(analyScene);
				break;
			}
		}
	}


    /// Probar a calibrar la kinect y corregir la diferencia entre las cámaras RGB y Depth
    if ( ( g_Depth.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) ) && ( g_Image.IsValid() )){
//            cout << "vista alternativa" << endl;
            g_Depth.GetAlternativeViewPointCap().SetViewPoint(g_Image);
    }
    else{
//        cout << " NO vista alternativa" << endl;
        g_Depth.GetAlternativeViewPointCap().ResetViewPoint();
    }



    if (g_Depth.IsCapabilitySupported(XN_CAPABILITY_USER_POSITION))
        cout << "depth capability user" << endl;
    if (g_User.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
        cout << "user capability skeleton" << endl;


//cout << "MyKinect::init" << endl;
    return 1;
}


/**
Lee el numero de Frames en el caso de ser un fichero .oni
*/
int MyKinect::readNumFrames(){

	XnStatus rc = XN_STATUS_OK;

   // para obtener el numero de frames de imagen y profundidad
    rc = player.SetRepeat(TRUE);

    if (rc != XN_STATUS_OK){
        printf("%s\n", xnGetStatusString(rc));
        return(-1);
    }
    rc = player.GetNumFrames(g_Depth.GetName(), numFramesDepth);
    if (rc != XN_STATUS_OK){
        printf("%s\n", xnGetStatusString(rc));
        return(-1);
    }
    rc = player.GetNumFrames(g_Image.GetName(), numFramesImage);
    if (rc != XN_STATUS_OK){
        printf("%s\n", xnGetStatusString(rc));
        return(-1);
    }
}


/**
Lee un frame de la kinect
*/
int MyKinect::readFrame(MyImage *imgMyImage)
{

	XnStatus rc = XN_STATUS_OK;
	EnumerationErrors errors;

	  rc = g_Context.WaitAnyUpdateAll(); // es el que va mejor!!!!!
//	  rc = g_Context.WaitAndUpdateAll();
//	  rc = g_Context.WaitNoneUpdateAll();  // con este no se visualiza nada!!!!!
//    rc = g_Context.WaitOneUpdateAll(g_Image);
//    rc = g_Context.WaitOneUpdateAll(g_Depth);

	if (rc != XN_STATUS_OK){
		printf("%s\n", xnGetStatusString(rc));
		return(-1);
	}
    if (g_Image.IsValid()){
//        cout <<  "IMG VALIDO" << endl;
        g_Image.GetMetaData(g_ImageMD);
    }
    if (g_Depth.IsValid()){
//        cout <<  "DEPTH VALIDO" << endl;
        g_Depth.GetMetaData(g_DepthMD);
    }
    if (g_IR.IsValid()){
//        cout <<  "IR VALIDO" << endl;
        g_IR.GetMetaData(g_irMD);
    }


    if (analyScene.IsValid()){
//        cout <<  "analyScene VALIDO" << endl;
        analyScene.GetMetaData(analySceneMD);
    }

    //process image data
    // Take current image
    XnRGB24Pixel* pImage =  const_cast <XnRGB24Pixel*> (g_Image.GetRGB24ImageMap());

    IplImage *imageAux= cvCreateImageHeader(cvSize(g_ImageMD.XRes(),g_ImageMD.YRes()),8,3);
    IplImage *iplImgFrame= cvCreateImage(cvSize(g_ImageMD.XRes(),g_ImageMD.YRes()),8,3);

    XnRGB24Pixel* pImageData = const_cast <XnRGB24Pixel*> (pImage);
    cvSetData(imageAux,pImageData, 640*3);
    cvCvtColor(imageAux, iplImgFrame, CV_RGB2BGR);

//    cvShowImage("RGB Dentro",iplImgFrame);
//    cvShowImage("RGB Dentro Aux",imageAux);


    //process depth data
    // Take current depth map
    XnDepthPixel* pDepthMap = const_cast <XnDepthPixel*> ( g_Depth.GetDepthMap());

    IplImage *iplDepthFrame= cvCreateImageHeader(cvSize(g_DepthMD.XRes(),g_DepthMD.YRes()),16,1);

    XnDepthPixel* pDepthData= const_cast <XnDepthPixel*> (pDepthMap);
    cvSetData(iplDepthFrame,pDepthData, 640*2);

//    cvShowImage("Depth Dentro Aux",iplDepthFrame);



    //actualizar objeto imgMyImage
    imgMyImage->setImg(iplImgFrame);
    imgMyImage->setDepth(iplDepthFrame);

//    imshow("myimagedentro",imgMyImage->getImg());
//    imshow("myimageDepth", imgMyImage->getShowDepth(0,10000000));
//    imshow("myimageDepth 0 255", imgMyImage->getShowDepth(0,255));


    //actualizar Frames
    imgMyImage->setFrameImg(g_ImageMD.FrameID());
    imgMyImage->setFrameDepth(g_DepthMD.FrameID());

//cout << "release" << endl;
    cvReleaseImageHeader(&imageAux);
    cvReleaseImage(&iplImgFrame);
    cvReleaseImageHeader(&iplDepthFrame);
//cout << "después release" << endl;

//cout << "MyKinect::readFrame" << endl;
    return 1;
}


/**
Escribe un frame a la kinect (para grabar ficheros .oni)
*/
int MyKinect::writeFrame(MyImage *imgMyImage){

	XnStatus rc = XN_STATUS_OK;
	EnumerationErrors errors;

	  rc = g_Context.WaitAnyUpdateAll(); // es el que va mejor!!!!!
//	  rc = g_Context.WaitAndUpdateAll();
//	  rc = g_Context.WaitNoneUpdateAll();  // con este no se visualiza nada!!!!!
//    rc = g_Context.WaitOneUpdateAll(g_Image);
//    rc = g_Context.WaitOneUpdateAll(g_Depth);

	if (rc != XN_STATUS_OK){
		printf("%s\n", xnGetStatusString(rc));
		return(-1);
	}
    if (g_Image.IsValid()){
//cout <<  "IMG VALIDO" << endl;
        g_Image.GetMetaData(g_ImageMD);
    }
    if (g_Depth.IsValid()){
//cout <<  "DEPTH VALIDO" << endl;
        g_Depth.GetMetaData(g_DepthMD);
    }
    if (mockg_Image.IsValid()){
//cout <<  "MOCK IMG VALIDO" << endl;
    }
    if (mockg_Depth.IsValid()){
//cout <<  "MOCK DEPTH VALIDO" << endl;
    }


    //process image data
    // hacer que se pueda modificar
    rc= g_ImageMD.MakeDataWritable();
	if (rc != XN_STATUS_OK){
		printf("g_ImageMD.MakeDataWritable(); - %s\n", xnGetStatusString(rc));
		return(-1);
	}

    // Take current image
    RGB24Map& imageMap = g_ImageMD.WritableRGB24Map();
	for (XnUInt32 y = 0; y < imageMap.YRes(); y++)
	{
		for (XnUInt32 x = 0; x < imageMap.XRes(); x++)
		{
			XnRGB24Pixel imagePixel;

			imagePixel.nBlue=imgMyImage->getValueB(x,y);
			imagePixel.nGreen=imgMyImage->getValueG(x,y);
			imagePixel.nRed=imgMyImage->getValueR(x,y);

			imageMap(x,y) = imagePixel;
		}
	}

    //escribir los datos "realmente"
    rc= mockg_Image.SetData(g_ImageMD);
	if (rc != XN_STATUS_OK){
		printf("mockg_Image.SetData(g_ImageMD); - %s\n", xnGetStatusString(rc));
		return(-1);
	}

    //process depth data
    // hacer que se pueda modificar
    rc= g_DepthMD.MakeDataWritable();
	if (rc != XN_STATUS_OK){
		printf("g_DepthMD.MakeDataWritable(); - %s\n", xnGetStatusString(rc));
		return(-1);
	}


    // Take current depth map
    DepthMap& depthMap = g_DepthMD.WritableDepthMap();
	for (XnUInt32 y = 0; y < depthMap.YRes(); y++)
	{
		for (XnUInt32 x = 0; x < depthMap.XRes(); x++)
		{
			//Punch vertical cut lines in the depth image
			depthMap(x,y) = imgMyImage->getValueDepth(x,y);
		}
	}

    //escribir los datos "realmente"
    rc= mockg_Depth.SetData(g_DepthMD);
	if (rc != XN_STATUS_OK){
		printf("mockg_Depth.SetData(g_DepthMD); - %s\n", xnGetStatusString(rc));
		return(-1);
	}

//cout << "MyKinect::writeFrame" << endl;
    return 1;
}

/**
* Modifica el valor de TILT de la kinect (utiliza freenect ya que a día de hoy no se puede con OpenNI)
*/
int MyKinect::setTilt(int arg_freenect_angle){


    freenect_angle= arg_freenect_angle;

    freenect_context *f_ctx;
    freenect_device *f_dev;

    if (freenect_angle > 30) {
        freenect_angle = 30;
        return 1;
    }

    //cerrar openni
    g_Context.Shutdown();
//    sleep(20);


    // abrir libfreenect
	while (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
//		return -1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
//	if (argc > 1)
//		user_device_number = atoi(argv[1]);

	if (nr_devices < 1)
		return -1;

	while (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		sleep(3);
//		return -1;
	}


//    freenect_sync_set_tilt_degs(14,0);
    cout << "poner el tilt" << endl;
    freenect_set_tilt_degs(f_dev, freenect_angle);

    cout << "cerrar kinect" << endl;
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);


	//volver a abrir openni
    if (init(fileXML) < 0){
       cout << "ERROR al iniciar la kinect con el fichero " << fileXML << endl;
       exit(-1);
    }
    if (initGenerators() < 0){
        cout << "ERROR al iniciar la Kinect" << endl;
        return -1;
    }
//cout << "MyKinect::setTilt" << endl;
}

/**
* Modifica el valor del LED de la kinect (utiliza freenect ya que a día de hoy no se puede con OpenNI)
*/
int MyKinect::setLed(freenect_led_options arg_led){

    led= arg_led;

    freenect_context *f_ctx;
    freenect_device *f_dev;


    //cerrar openni
    g_Context.Shutdown();
//    sleep(20);


    // abrir libfreenect
	while (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		sleep(3);
//		return -1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
//	if (argc > 1)
//		user_device_number = atoi(argv[1]);

	if (nr_devices < 1)
		return -1;

	while (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
//		return -1;
	}


//    freenect_sync_set_tilt_degs(14,0);
    cout << "poner el led" << endl;
    freenect_set_led(f_dev, led);

    cout << "cerrar kinect" << endl;
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);


	//volver a abrir openni y capturar algo para que no comience la captura de nuevo
    if (init(fileXML) < 0){
       cout << "ERROR al iniciar la kinect con el fichero " << fileXML << endl;
       exit(-1);
    }
    if (initGenerators() < 0){
        cout << "ERROR al iniciar la Kinect" << endl;
        return -1;
    }
//cout << "MyKinect::setLed" << endl;
}


/**
* Inicializa y realiza la grabación de la kinect en un fichero ONI
*/
int MyKinect::saveONIFile(String outputName){

    XnStatus rc= XN_STATUS_OK;
    NodeInfoList recorderList;
    rc= g_Context.EnumerateProductionTrees(XN_NODE_TYPE_RECORDER,NULL,recorderList);
    if (rc != XN_STATUS_OK){
        printf("ERROR creación de 'enumerate recorders' %s\n", xnGetStatusString(rc));
        return(-1);
    }
    // coger el primero de la lista de "recorder"
    NodeInfo chosen = *recorderList.Begin();

    rc= g_Context.CreateProductionTree(chosen);
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de 'recorder' %s\n", xnGetStatusString(rc));
        return(-1);
    }

    pRecorder= new Recorder();
    rc= chosen.GetInstance(*(pRecorder));
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de 'get recorder instance' %s\n", xnGetStatusString(rc));
        return(-1);
    }

    rc= pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, outputName.c_str());
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de fichero .oni (%s) %s\n", outputName.c_str(), xnGetStatusString(rc));
        return(-1);
    }

    // preparar para grabar....
    if (g_Device.IsValid()){
        (debug > 1) ? cout << "graba device" << endl : cout << "";
        rc= pRecorder->AddNodeToRecording(g_Device,XN_CODEC_UNCOMPRESSED);
        if (rc != XN_STATUS_OK){
            printf("ERROR add device node %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }
    if (g_Depth.IsValid()){
        (debug > 1) ? cout << "graba depth" << endl : cout << "";
//        rc= pRecorder->AddNodeToRecording(g_Depth, XN_CODEC_UNCOMPRESSED);
        rc= pRecorder->AddNodeToRecording(g_Depth, XN_CODEC_16Z_EMB_TABLES);
        if (rc != XN_STATUS_OK){
            printf("ERROR add depth node %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }
    if (g_Image.IsValid()){
        (debug > 1) ? cout << "graba image" << endl : cout << "";
//        rc= pRecorder->AddNodeToRecording(g_Image, XN_CODEC_UNCOMPRESSED);
        rc= pRecorder->AddNodeToRecording(g_Image, XN_CODEC_JPEG);
        if (rc != XN_STATUS_OK){
            printf("ERROR add image node %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }
    // es para intentar grabar el usuario y demás en el fichero .oni ... por ahora no se graba....
    if (analyScene.IsValid()){
        (debug > 1) ? cout << "graba scene" << endl : cout << "";
        rc= pRecorder->AddNodeToRecording(analyScene, XN_CODEC_UNCOMPRESSED);
        if (rc != XN_STATUS_OK){
            printf("ERROR add analysis Scene %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }


    (debug > 1) ? cout << "MyKinect::saveONIFile" << endl : cout << "";
}

/**
* Inicializa y realiza la grabación "mock" (modificada) de un fichero ONI a otro
*/
int MyKinect::saveMockONIFile(String outputName){

    XnStatus rc= XN_STATUS_OK;
    NodeInfoList recorderList;
    rc= g_Context.EnumerateProductionTrees(XN_NODE_TYPE_RECORDER,NULL,recorderList);
    if (rc != XN_STATUS_OK){
        printf("ERROR creación de 'enumerate recorders' %s\n", xnGetStatusString(rc));
        return(-1);
    }
    // coger el primero de la lista de "recorder"
    NodeInfo chosen = *recorderList.Begin();

    rc= g_Context.CreateProductionTree(chosen);
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de 'recorder' %s\n", xnGetStatusString(rc));
        return(-1);
    }

    pRecorder= new Recorder();
    rc= chosen.GetInstance(*(pRecorder));
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de 'get recorder instance' %s\n", xnGetStatusString(rc));
        return(-1);
    }

    rc= pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, outputName.c_str());
    if (rc != XN_STATUS_OK){
        printf("ERROR creacion de fichero .oni (%s) %s\n", outputName.c_str(), xnGetStatusString(rc));
        return(-1);
    }

    rc = mockg_Depth.CreateBasedOn(g_Depth);
    if (rc != XN_STATUS_OK){
        printf("ERROR crear mockg_Depth.CreateBasedOn(g_Depth) %s\n", xnGetStatusString(rc));
        return(-1);
    }
    if (mockg_Depth.IsValid()){
        (debug > 1) ? cout << "graba depth" << endl : cout << "";
        rc= pRecorder->AddNodeToRecording(mockg_Depth, XN_CODEC_16Z_EMB_TABLES);
        if (rc != XN_STATUS_OK){
            printf("ERROR add depth node %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }

    rc = mockg_Image.CreateBasedOn(g_Image);
    if (rc != XN_STATUS_OK){
        printf("ERROR crear mockg_Image.CreateBasedOn(g_Image) %s\n", xnGetStatusString(rc));
        return(-1);
    }
    if (mockg_Image.IsValid()){
        (debug > 1) ? cout << "graba image" << endl : cout << "";
        rc= pRecorder->AddNodeToRecording(mockg_Image, XN_CODEC_JPEG);
        if (rc != XN_STATUS_OK){
            printf("ERROR add image node %s\n", xnGetStatusString(rc));
            return(-1);
        }
    }

    (debug > 1) ? cout << "MyKinect::saveMockONIFile" << endl : cout << "";
}



/**
* Realiza la grabación de cada frame
*/
int MyKinect::record(){
    XnStatus rc= XN_STATUS_OK;
    rc= pRecorder->Record();
//cout << "empieza a grabar..." << endl;
    if (rc != XN_STATUS_OK){
        printf("ERROR pRecorder->Record() %s\n", xnGetStatusString(rc));
        return(-1);
    }

//cout << "frame grabado: " << g_Image.GetFrameID() << endl;

//	  rc = g_Context.WaitAnyUpdateAll();
//    rc = g_Context.WaitAndUpdateAll();
//	  rc = g_Context.WaitNoneUpdateAll();
//    rc = g_Context.WaitOneUpdateAll(g_Image);
//    if (rc != XN_STATUS_OK){
//        printf("ERROR pRecorder->Record() %s\n", xnGetStatusString(rc));
//        return(-1);
//    }
//cout << "MyKinect::record" << endl;
}


/**
* Detiene la grabación
*/
int MyKinect::stopRecord(){


cout << "frame grabado: " << g_Image.GetFrameID() << endl;

    pRecorder->Release();
    delete pRecorder;
//cout << "MyKinect::stopRecord" << endl;
}


/**
* get numFrmesImage
*/
XnUInt32 MyKinect::getnumFramesImage(){
    return numFramesImage;
}
/**
* get numFrmesDepth
*/
XnUInt32 MyKinect::getnumFramesDepth(){
    return numFramesDepth;
}

/**
* get g_Context
*/
Context MyKinect::getg_Context(){
    return g_Context;
}
/**
* get g_Device
*/
Device MyKinect::getg_Device(){
    return g_Device;
}
/**
* get g_Depth
*/
DepthGenerator MyKinect::getg_Depth(){
    return g_Depth;
}
/**
* get g_Image
*/
ImageGenerator MyKinect::getg_Image(){
    return g_Image;
}
/**
* get g_IR
*/
IRGenerator MyKinect::getg_IR(){
    return g_IR;
}
/**
* get g_DepthMD
*/
const DepthMetaData* MyKinect::getg_DepthMD(){
    return &g_DepthMD;
}
/**
* get g_ImageMD
*/
const ImageMetaData* MyKinect::getg_ImageMD(){
    return &g_ImageMD;
}
/**
* get g_irMD
*/
const IRMetaData* MyKinect::getg_irMD(){
    return &g_irMD;
}
/**
* get g_User
*/
UserGenerator MyKinect::getg_User(){
    return g_User;
}
/**
* get analyScene
*/
SceneAnalyzer MyKinect::getanalyScene(){
    return analyScene;
}
/**
* get analySceneMD
*/
const SceneMetaData* MyKinect::getanalySceneMD(){
    return &analySceneMD;
}
/**
* get freenect_angle
*/
int MyKinect::getfreenect_angle(){
    return freenect_angle;
}
/**
* get led
*/
freenect_led_options MyKinect::getled(){
    return led;
}
/**
* get fileXML
*/
String MyKinect::getfileXML(){
    return fileXML;
}


