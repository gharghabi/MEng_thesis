#include "MyImage.h"

MyImage::MyImage(){

    img=Mat::zeros(100,100,CV_8UC3);
    Mat aux(img.rows,img.cols,CV_8UC1);
    cvtColor(img,aux,CV_BGR2GRAY);
    aux.copyTo(imgGray);

    depth= Mat::zeros(100,100,CV_16UC1);

    frameImg= -1;
    frameDepth= -1;
    debug=0;

//cout << "MyImage contructor" << endl;
}

MyImage::~MyImage(){

//cout << "MyImage destructor" << endl;
}

/** Default constructor */
MyImage::MyImage(Mat image){

    if (IplImage(image).nChannels == 3){
        image.copyTo(img);
        Mat aux(img.rows,img.cols,CV_8UC1);
        cvtColor(img,aux,CV_BGR2GRAY);
        aux.copyTo(imgGray);

//cout << "mat const RGB" << endl;
    }
    else{
        image.copyTo(imgGray);
        Mat aux(imgGray.rows,imgGray.cols,CV_8UC3);
        cvtColor(imgGray,aux,CV_GRAY2BGR);
        aux.copyTo(img);

//cout << "mat const Gray" << endl;
    }

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);

    frameImg= -1;
    frameDepth= -1;
    debug=0;

//cout << "MyImage constructor Mat" << endl;
}
/** Default constructor */
MyImage::MyImage(IplImage image){

    if (image.nChannels == 3){
        Mat aux(&image);
        aux.copyTo(img);

        Mat aux2(img.rows,img.cols,CV_8UC1);
        cvtColor(img,aux2,CV_BGR2GRAY);
        aux2.copyTo(imgGray);


//cout << "ipl const RGB" << endl;
    }
    else{
        Mat aux(&image);
        aux.copyTo(imgGray);

        Mat aux2(imgGray.rows,imgGray.cols,CV_8UC3);
        cvtColor(imgGray,aux2,CV_GRAY2BGR);
        aux2.copyTo(img);

//cout << "ipl const GRAY" << endl;
    }

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);


    frameImg= -1;
    frameDepth= -1;
    debug=0;

//cout << "MyImage constructor IplImage" << endl;
}

/** Constructor de copia */
MyImage::MyImage(const MyImage &image){

    image.img.copyTo(img);
    image.imgGray.copyTo(imgGray);
    image.depth.copyTo(depth);

    frameImg= image.frameImg;
    frameDepth= image.frameDepth;
    debug= image.debug;

//cout << "MyImage constructor copy" << endl;
}
/** Operador de asignacion por copia */
MyImage & MyImage::operator=(const MyImage &image){

    image.img.copyTo(img);
    image.imgGray.copyTo(imgGray);
    image.depth.copyTo(depth);

    frameImg= image.frameImg;
    frameDepth= image.frameDepth;
    debug= image.debug;

//cout << "MyImage operator =" << endl;
    return *this;
}

/** Access img
 * \return The current value of img
 */
Mat MyImage::getImg(){

//cout << "mat img get" << endl;

    return img;
}
/** Set img
 * \param val New value to set
 */
void MyImage::setImg(Mat val){

    if (IplImage(val).nChannels == 3){
        val.copyTo(img);
        Mat aux(img.rows,img.cols,CV_8UC1);
        cvtColor(img,aux,CV_BGR2GRAY);
        aux.copyTo(imgGray);
//cout << "mat set RGB" << endl;
    }
    else{
        val.copyTo(imgGray);
        Mat aux(imgGray.rows,imgGray.cols,CV_8UC3);
        cvtColor(imgGray,aux,CV_GRAY2BGR);
        aux.copyTo(img);
//cout << "mat const Gray" << endl;
    }

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);

//cout << "MyImage mat setImg" << endl;
}

/** Access depth
 * \return The current value of img
 */
Mat MyImage::getDepth(){

//cout << "mat imgDepth get" << endl;

    return depth;
}
/** Access depth tipo Mat
 * \param min valor mínimo a considerar
 * \param max valor máximo a considerar
 * \return The current value of img
 */
Mat MyImage::getShowDepth(int min, int max){

    int fil=img.rows, col=img.cols;
    Mat img= Mat(fil,col,CV_8UC1);


    for (int i=0; i < fil; i++){
        for (int j=0; j < col; j++){
            if (depth.at<XnUInt16>(i,j) < min){
                img.at<uchar>(i,j)= min;
//cout << " < min " << endl;
            }
            else if (depth.at<XnUInt16>(i,j) > max){
                img.at<uchar>(i,j)= max;
//cout << " > max " << endl;
            }
            else{
                img.at<uchar>(i,j)= (uchar) depth.at<XnUInt16>(i,j);
            }
        }
    }

    return img;
}
/** Set depth
 * \param val New value to set
 */
void MyImage::setDepth(Mat val){

    val.copyTo(depth);

//cout << "MyImage depth set" << endl;
}

/** Access imgGray
 * \return The current value of imgGray
 */
Mat MyImage::getImgGray(){

//cout << "mat gray get" << endl;

    return imgGray;
}
/** Set imgGray
 * \param val New value to set
 */
void MyImage::setImgGray(Mat val){

    val.copyTo(imgGray);

    Mat aux(imgGray.rows,imgGray.cols,CV_8UC3);
    cvtColor(imgGray,aux,CV_GRAY2BGR);
    aux.copyTo(img);

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);

//cout << "MyImage mat set gray" << endl;
}
/** Access img
 * \return The current value of img
 * @note Puntero a la imagen, OJO se puede modificar desde fuera
 */

IplImage MyImage::getIplImg(){

//cout << "ipl img get" << endl;

    return IplImage(img);
}
/** Set img
 * \param val New value to set
 */
void MyImage::setImg(IplImage val){

    if (val.nChannels == 3){
        Mat aux(&val);
        aux.copyTo(img);

        Mat aux2(img.rows,img.cols,CV_8UC1);
        cvtColor(img,aux2,CV_BGR2GRAY);
        aux2.copyTo(imgGray);
//cout << "ipl set RGB" << endl;
    }
    else{
        Mat aux(&val);
        aux.copyTo(imgGray);

        Mat aux2(imgGray.rows,imgGray.cols,CV_8UC3);
        cvtColor(imgGray,aux2,CV_GRAY2BGR);
        aux2.copyTo(img);
//cout << "ipl set GRAY" << endl;
    }

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);

//cout << "MyImage ipl set" << endl;
}
/** Access imgGray
 * \return The current value of imgGray
 * @note Puntero a la imagen, OJO se puede modificar desde fuera
 */
IplImage MyImage::getIplImgGray(){

//cout << "ipl img gray get" << endl;

    return IplImage(imgGray);
}
/** Set imgGray
 * \param val New value to set
 */
void MyImage::setImgGray(IplImage val){

    Mat aux(&val);
    aux.copyTo(imgGray);

    Mat aux2(imgGray.rows,imgGray.cols,CV_8UC3);
    cvtColor(imgGray,aux2,CV_GRAY2BGR);
    aux2.copyTo(img);

    depth= Mat::zeros(img.rows,img.cols,CV_16UC1);

//cout << "MyImage ipl img gray set" << endl;
}

/** Access depth
 * \return The current value of depth
 * @note Puntero a la imagen, OJO se puede modificar desde fuera
 */
IplImage MyImage::getIplDepth(){

//cout << "ipl depth get" << endl;

    return IplImage(depth);
}

/** Set depth
 * \param val New value to set
 */
void MyImage::setDepth(IplImage val){

    if (img.cols != val.width || img.rows != val.height){
        cout << "El tamaño de la imagen de color y la imagen de profundidad debe ser el mismo" << endl;
        CV_Assert(!(img.cols != val.width || img.rows != val.height));
        exit(1);
    }

    Mat aux(&val);
    aux.copyTo(depth);

//cout << "MyImage ipl depth set" << endl;
}

/** Get width
 * @return image width
 */
int MyImage::getWidth(){
    return img.cols;
}
 /** get Height
 * @return image height
 */
int MyImage::getHeight(){
    return img.rows;
}

 /** get Value channel R
 * @param x pos X del pixel en el canal R
 * @param y pos Y del pixel en el canal R
 * @return int valor del pixel (x,y) del canal R
 */
int MyImage::getValueR(int x, int y){
    Vec3b BGR=img.at<Vec3b>(y,x);
    return (int)BGR[2];
}
 /** set Value channel R
 * @param x pos X del pixel en el canal R
 * @param y pos Y del pixel en el canal R
 * @param valor del pixel (x,y) del canal R
 */
void MyImage::setValueR(int x, int y, int v){
    Vec3b BGR=img.at<Vec3b>(y,x);
    BGR[2]=saturate_cast<uchar>(v);
    img.at<Vec3b>(y,x)=BGR;
    cvtColor(img,imgGray,CV_BGR2GRAY);
}
 /** get Value channel G
 * @param x pos X del pixel en el canal G
 * @param y pos Y del pixel en el canal G
 * @return int valor del pixel (x,y) del canal G
 */
int MyImage::getValueG(int x, int y){
    Vec3b BGR=img.at<Vec3b>(y,x);
    return (int)BGR[1];
}
 /** set Value channel G
 * @param x pos X del pixel en el canal G
 * @param y pos Y del pixel en el canal G
 * @param valor del pixel (x,y) del canal G
 */
void MyImage::setValueG(int x, int y, int v){
    Vec3b BGR=img.at<Vec3b>(y,x);
    BGR[1]=saturate_cast<uchar>(v);
    img.at<Vec3b>(y,x)=BGR;
    cvtColor(img,imgGray,CV_BGR2GRAY);
}
 /** get Value channel B
 * @param x pos X del pixel en el canal B
 * @param y pos Y del pixel en el canal B
 * @return int valor del pixel (x,y) del canal B
 */
int MyImage::getValueB(int x, int y){
    Vec3b BGR=img.at<Vec3b>(y,x);
    return (int)BGR[0];
}
 /** set Value channel B
 * @param x pos X del pixel en el canal B
 * @param y pos Y del pixel en el canal B
 * @param valor del pixel (x,y) del canal B
 */
void MyImage::setValueB(int x, int y, int v){
    Vec3b BGR=img.at<Vec3b>(y,x);
    BGR[0]=saturate_cast<uchar>(v);
    img.at<Vec3b>(y,x)=BGR;
    cvtColor(img,imgGray,CV_BGR2GRAY);
}
 /** get Value channel Gray
 * @param x pos X del pixel en el canal Gray
 * @param y pos Y del pixel en el canal Gray
 * @return int valor del pixel (x,y) del canal Gray
 */
int MyImage::getValueGray(int x, int y){
    return (int) imgGray.at<uchar>(y,x);
}
 /** set Value channel Gray
 * @param x pos X del pixel en el canal Gray
 * @param y pos Y del pixel en el canal Gray
 * @param valor del pixel (x,y) del canal Gray
 */
void MyImage::setValueGray(int x, int y, int v){
    imgGray.at<uchar>(y,x)=saturate_cast<uchar>(v);
    cvtColor(imgGray,img,CV_GRAY2BGR);
}

 /** get Value Depth
 * @param x pos X del pixel en depth
 * @param y pos Y del pixel en depth
 * @return int valor del pixel (x,y) de depth
 */
int MyImage::getValueDepth(int x, int y){
    XnUInt16 aux= depth.at<XnUInt16>(y,x);
    return (int) aux;
}
 /** set Value channel Depth
 * @param x pos X del pixel en depth
 * @param y pos Y del pixel en depth
 * @param valor del pixel (x,y) de depth
 */
void MyImage::setValueDepth(int x, int y, int v){
    depth.at<XnUInt16>(y,x)= saturate_cast<XnUInt16>(v);
}

 /** get frame value of image
  * @return int frame value of image
 */
int MyImage::getFrameImg(){
    return frameImg;
}
 /** set frame value of image
  * @param v int frame value of image
 */
void MyImage::setFrameImg(int v){
    frameImg= v;
}
 /** get frame value of depth
  * @return int frame value of depth
 */
int MyImage::getFrameDepth(){
    return frameDepth;
}
 /** set frame value of depth
  * @param v int frame value of depth
 */
void MyImage::setFrameDepth(int v){
    frameDepth= v;
}
