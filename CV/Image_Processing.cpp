#include "mainwindow.h"
#include "math.h"
#include "ui_mainwindow.h"
#include <QtGui>
#include <iostream>

/* Author Kyle Lashbrook
 * Class EE 576 CV
 /*



/***********************************************************************
  This is the only file you need to change for your assignment. The
  other files control the UI (in case you want to make changes.)
************************************************************************/

/***********************************************************************
  The first eight functions provide example code to help get you started
************************************************************************/


// Convert an image to grayscale
void MainWindow::BlackWhiteImage(QImage *image)
{
    for(int r=0;r<image->height();r++)
        for(int c=0;c<image->width();c++)
        {
            QRgb pixel = image->pixel(c, r);
            double red = (double) qRed(pixel);
            double green = (double) qGreen(pixel);
            double blue = (double) qBlue(pixel);

            // Compute intensity from colors - these are common weights
            double intensity = 0.3*red + 0.6*green + 0.1*blue;

            image->setPixel(c, r, qRgb( (int) intensity, (int) intensity, (int) intensity));
        }
}

// Add random noise to the image
void MainWindow::AddNoise(QImage *image, double mag, bool colorNoise)
{
    int noiseMag = mag*2;

    for(int r=0;r<image->height();r++)
        for(int c=0;c<image->width();c++)
        {
            QRgb pixel = image->pixel(c, r);
            int red = qRed(pixel), green = qGreen(pixel), blue = qBlue(pixel);

            // If colorNoise, add color independently to each channel
            if(colorNoise)
            {
                red += rand()%noiseMag - noiseMag/2;
                green += rand()%noiseMag - noiseMag/2;
                blue += rand()%noiseMag - noiseMag/2;
            }
            // otherwise add the same amount of noise to each channel
            else
            {
                int noise = rand()%noiseMag - noiseMag/2;
                red += noise; green += noise; blue += noise;
            }
            image->setPixel(c, r, qRgb(max(0, min(255, red)), max(0, min(255, green)), max(0, min(255, blue))));
        }
}

// Downsample the image by 1/2
void MainWindow::HalfImage(QImage &image)
{
    int w = image.width();
    int h = image.height();
    QImage buffer = image.copy();

    // Reduce the image size.
    image = QImage(w/2, h/2, QImage::Format_RGB32);

    // Copy every other pixel
    for(int r=0;r<h/2;r++)
        for(int c=0;c<w/2;c++)
             image.setPixel(c, r, buffer.pixel(c*2, r*2));
}

// Round float values to the nearest integer values and make sure the value lies in the range [0,255]
QRgb restrictColor(double red, double green, double blue)
{
    int r = (int)(floor(red+0.5));
    int g = (int)(floor(green+0.5));
    int b = (int)(floor(blue+0.5));

    return qRgb(max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)));
}

// Normalize the values of the kernel to sum-to-one
void NormalizeKernel(double *kernel, int kernelWidth, int kernelHeight)
{
    double denom = 0.000001; int i;
    for(i=0; i<kernelWidth*kernelHeight; i++)
        denom += kernel[i];
    for(i=0; i<kernelWidth*kernelHeight; i++)
        kernel[i] /= denom;
}

// Here is an example of blurring an image using a mean or box filter with the specified radius.
// This could be implemented using separable filters to make it much more efficient, but it's not done here.
// Note: This function is written using QImage form of the input image. But all other functions later use the double form
void MainWindow::MeanBlurImage(QImage *image, int radius)
{
    if(radius == 0)
        return;
    int size = 2*radius + 1; // This is the size of the kernel

    // Note: You can access the width and height using 'imageWidth' and 'imageHeight' respectively in the functions you write
    int w = image->width();
    int h = image->height();

    // Create a buffer image so we're not reading and writing to the same image during filtering.
    // This creates an image of size (w + 2*radius, h + 2*radius) with black borders (zero-padding)
    QImage buffer = image->copy(-radius, -radius, w + 2*radius, h + 2*radius);

    // Compute the kernel to convolve with the image
    double *kernel = new double [size*size];

    for(int i=0;i<size*size;i++)
        kernel[i] = 1.0;

    // Make sure kernel sums to 1
    NormalizeKernel(kernel, size, size);

    // For each pixel in the image...
    for(int r=0;r<h;r++)
    {
        for(int c=0;c<w;c++)
        {
            double rgb[3];
            rgb[0] = rgb[1] = rgb[2] = 0.0;

            // Convolve the kernel at each pixel
            for(int rd=-radius;rd<=radius;rd++)
                for(int cd=-radius;cd<=radius;cd++)
                {
                     // Get the pixel value
                     //For the functions you write, check the ConvertQImage2Double function to see how to get the pixel value
                     QRgb pixel = buffer.pixel(c + cd + radius, r + rd + radius);

                     // Get the value of the kernel
                     double weight = kernel[(rd + radius)*size + cd + radius];

                     rgb[0] += weight*(double) qRed(pixel);
                     rgb[1] += weight*(double) qGreen(pixel);
                     rgb[2] += weight*(double) qBlue(pixel);
                }
            // Store the pixel in the image to be returned
            // You need to store the RGB values in the double form of the image
            image->setPixel(c, r, restrictColor(rgb[0],rgb[1],rgb[2]));
        }
    }
    // Clean up (use this carefully)
    delete[] kernel;
}

// Convert QImage to a matrix of size (imageWidth*imageHeight)*3 having double values
void MainWindow::ConvertQImage2Double(QImage image)
{
    // Global variables to access image width and height
    imageWidth = image.width();
    imageHeight = image.height();

    // Initialize the global matrix holding the image
    // This is how you will be creating a copy of the original image inside a function
    // Note: 'Image' is of type 'double**' and is declared in the header file (hence global variable)
    // So, when you create a copy (say buffer), write "double** buffer = new double ....."
    Image = new double* [imageWidth*imageHeight];
    for (int i = 0; i < imageWidth*imageHeight; i++)
            Image[i] = new double[3];

    // For each pixel
    for (int r = 0; r < imageHeight; r++)
        for (int c = 0; c < imageWidth; c++)
        {
            // Get a pixel from the QImage form of the image
            QRgb pixel = image.pixel(c,r);

            // Assign the red, green and blue channel values to the 0, 1 and 2 channels of the double form of the image respectively
            Image[r*imageWidth+c][0] = (double) qRed(pixel);
            Image[r*imageWidth+c][1] = (double) qGreen(pixel);
            Image[r*imageWidth+c][2] = (double) qBlue(pixel);
        }
}

// Convert the matrix form of the image back to QImage for display
void MainWindow::ConvertDouble2QImage(QImage *image)
{
    for (int r = 0; r < imageHeight; r++)
        for (int c = 0; c < imageWidth; c++)
            image->setPixel(c, r, restrictColor(Image[r*imageWidth+c][0], Image[r*imageWidth+c][1], Image[r*imageWidth+c][2]));
}


/**************************************************
 TIME TO WRITE CODE
**************************************************/

/**************************************************
 TASK 1
**************************************************/

// Convolve the image with the kernel
void MainWindow::Convolution(double** image, double *kernel, int kernelWidth, int kernelHeight, bool add)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * kernel: 1-D array of kernel values
 * kernelWidth: width of the kernel
 * kernelHeight: height of the kernel
 * add: a boolean variable (taking values true or false)
*/
{
    //Zero-pad the buffer image based on the kernel height and width
       int edge_h = kernelHeight/2;
       int edge_w = kernelWidth/2;
       int height = imageHeight + 2*edge_h;
       int width = imageWidth + 2*edge_w;
       int size = (width)*(height);
       //Create a buffer image
       double** buffer = new double* [size];
       for (int j = 0; j < size; j++)
               buffer[j] = new double[3];

     /***********************Original Zero Padding For Black Borders************************/


//pad border of buffer with zeros
//       for (int r = 0; r < height; r++)
//           for (int c = 0; c < width; c++)
//           {
//               if ((r < edge_h) || (r >= height - edge_h) || (c < edge_w) || (c >= width - edge_w))
//               {
//                   buffer[r*width+c][0] = 0.0;
//                   buffer[r*width+c][1] = 0.0;
//                   buffer[r*width+c][2] = 0.0;
//               }
//               else
//               {
//               buffer[r*width+c][0] = image[(r - edge_h)*imageWidth+(c - edge_w)][0];
//               buffer[r*width+c][1] = image[(r - edge_h)*imageWidth+(c -edge_w)][1];
//               buffer[r*width+c][2] = image[(r - edge_h)*imageWidth+(c - edge_w)][2];
//               }
//           }
       /* Above is the looping to implement black borders, below is for The extra credit implementing Extended Borders!!!
                                         Extra Credit                     */

          for (int r = 0; r < height; r++)
          {
              for (int c = 0; c < width; c++)
              {
                  // Left
                  if (c <  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c][0] = image[(r-edge_h)*imageWidth][0];
                      buffer[r*width+c][1] = image[(r-edge_h)*imageWidth][1];
                      buffer[r*width+c][2] = image[(r-edge_h)*imageWidth][2];
                  }
                  // Right
                  else if (c >= width -  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c][0] = image[(r-edge_h)*imageWidth+(imageWidth-1)][0];
                      buffer[r*width+c][1] = image[(r-edge_h)*imageWidth+(imageWidth-1)][1];
                      buffer[r*width+c][2] = image[(r-edge_h)*imageWidth+(imageWidth-1)][2];
                  }
                  // Center (Image)
                  else if (c >=  edge_w && c < width -  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c][0] = image[(r - edge_h)*imageWidth+(c -  edge_w)][0];
                      buffer[r*width+c][1] = image[(r - edge_h)*imageWidth+(c -  edge_w)][1];
                      buffer[r*width+c][2] = image[(r - edge_h)*imageWidth+(c -  edge_w)][2];
                  }
              }
          }
          for (int r = 0; r < height; r++)
          {
              for (int c = 0; c < width; c++)
              {
                  // Bottom
                  if (r >= height - edge_h)
                  {
                      buffer[r*width+c][0] = buffer[(height-edge_h-1)*width+c][0];
                      buffer[r*width+c][1] = buffer[(height-edge_h-1)*width+c][1];
                      buffer[r*width+c][2] = buffer[(height-edge_h-1)*width+c][2];
                  }
                  // Top
                  else if(r < edge_h)
                  {
                      buffer[r*width+c][0] = buffer[(edge_h)*width+c][0];
                      buffer[r*width+c][1] = buffer[(edge_h)*width+c][1];
                      buffer[r*width+c][2] = buffer[(edge_h)*width+c][2];
                  }
              }
          }


       // For each pixel in the image...
       for(int r = 0;r < imageHeight;r++)
       {
           for(int c = 0;c < imageWidth;c++)
           {
               image[r*imageWidth+c][0] = 0.0;
               image[r*imageWidth+c][1] = 0.0;
               image[r*imageWidth+c][2] = 0.0;

               // Convolve the kernel at each pixel
               for(int rd=-edge_h;rd<= edge_h;rd++)
                   for(int cd=-edge_w;cd<= edge_w;cd++)
                   {

                        // Get the value of the kernel
                       double weight = kernel[(rd + edge_h)*kernelWidth + cd + edge_w];

                       image[r*imageWidth+c][0] += weight*(double) buffer[(r + rd + edge_h)*width + (c + cd + edge_w)][0];
                       image[r*imageWidth+c][1] += weight*(double) buffer[(r + rd + edge_h)*width + (c + cd + edge_w)][1];
                       image[r*imageWidth+c][2] += weight*(double) buffer[(r + rd + edge_h)*width + (c + cd + edge_w)][2];

                   }
           }
       }
if(add==true){
  for(int j=0;j<imageWidth*imageHeight;j++){

  image[j][0]=image[j][0]+128;
    image[j][1]=image[j][1]+128;
      image[j][2]=image[j][2]+128;

  }



   }
}
   /**************************************************
    TASK 2
   **************************************************/

   // Apply the 2-D Gaussian kernel on an image to blur it
   void MainWindow::GaussianBlurImage(double** image, double sigma)
   /*
    * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
    * sigma: standard deviation for the Gaussian kernel
   */
   {
       int radius = (int)(ceil(3* sigma));
       int sizeW = 2*radius +1;
       int sizeH = 2*radius + 1;
       // Compute the kernel to convolve with the image
       double *kernel = new double [sizeW*sizeH]; //vector form

       for (int r = 0; r < sizeH; r++)
           for (int c = 0; c <  sizeW; c++)
          //Gaussian Equation
             kernel[r*sizeW + c] = (1/(2*M_PI*sigma*sigma))*pow(M_E, -((r-radius)*(r-radius) + (c-radius)*(c-radius))/(2*sigma*sigma));
       // Make sure kernel sums to 1
       NormalizeKernel(kernel, sizeW, sizeH);
       Convolution(image, kernel, sizeW, sizeH, false);
   }

/**************************************************
 TASK 3
**************************************************/

// Perform the Gaussian Blur first in the horizontal direction and then in the vertical direction
void MainWindow::SeparableGaussianBlurImage(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    int radius = (int)(ceil(3* sigma));
    int sizeW = 2*radius +1;
    int sizeH = 2*radius + 1;
    // Compute the kernel to convolve with the image
    double *kernel = new double [sizeW*sizeH]; //vector form

    for (int r = 0; r < sizeW; r++)
        for (int c = 0; c <  1; c++)
       //Gaussian for seperate rows
    kernel[r] = (1/(sqrt(2*M_PI)*sigma))*pow(M_E, -((r-radius)*(r-radius))/(2*sigma*sigma));

    // Make sure kernel sums to 1
    NormalizeKernel(kernel, sizeW, 1);
    Convolution(image, kernel, sizeW,1, false);


    for (int r = 0; r < 1; r++)
        for (int c = 0; c <  sizeH; c++)
            //Gaussian for Columns
     kernel[c] = (1/(sqrt(2*M_PI)*sigma))*pow(M_E, -((c-radius)*(c-radius))/(2*sigma*sigma));

    // Make sure kernel sums to 1
    NormalizeKernel(kernel, 1, sizeH);
    Convolution(image, kernel, 1, sizeH, false);
}

/********** TASK 4 (a) **********/

// Compute the First derivative of an image along the horizontal direction and then apply Gaussian blur.
void MainWindow::FirstDerivImage_x(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    int sizeW = 3;
    int sizeH = 1;
    // Compute the kernel to convolve with the image
    double *kernel = new double [sizeW*sizeH]{-1,0,1}; //vector form
      Convolution(image, kernel, sizeW, sizeH, true);
        SeparableGaussianBlurImage(image, sigma);
}

/********** TASK 4 (b) **********/

// Compute the First derivative of an image along the vertical direction and then apply Gaussian blur.
void MainWindow::FirstDerivImage_y(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{


    int sizeW = 1;
    int sizeH = 3;
    // Compute the kernel to convolve with the image
    double *kernel = new double [sizeW*sizeH]{-1,0,1}; //vector form
      Convolution(image, kernel, sizeW, sizeH, true);
        SeparableGaussianBlurImage(image, sigma);
}

/********** TASK 4 (c) **********/

// Compute the Second derivative of an image using the Laplacian operator and then apply Gaussian blur
void MainWindow::SecondDerivImage(double** image, double sigma)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
*/
{
    int sizeW = 3;
    int sizeH = 3;

    // Compute the kernel to convolve with the image

    double *kernel = new double [sizeW*sizeH]{0,1,0,1,-4,1,0,1,0}; //vector form


      Convolution(image, kernel, sizeW, sizeH, true);

      SeparableGaussianBlurImage(image, sigma);
}

/**************************************************
 TASK 5
**************************************************/

// Sharpen an image by subtracting the image's second derivative from the original image
void MainWindow::SharpenImage(double** image, double sigma, double alpha)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * sigma: standard deviation for the Gaussian kernel
 * alpha: constant by which the second derivative image is to be multiplied to before subtracting it from the original image
*/
{
           //Create new buffer copy of image
    double** buffer = new double* [imageHeight*imageWidth];
    for (int j = 0; j < imageHeight*imageWidth; j++)
            buffer[j] = new double[3];

    for (int j =0;j<3;j++){
    for (int i =0;i<imageWidth*imageHeight;i++)

        buffer[i][j]=image[i][j];

        }
    //buffer is now secondderiv of image
    SecondDerivImage(buffer,sigma);
//compute sharpen image-alpha*buffer-128
for(int r = 0;r < imageHeight;r++)
{
    for(int c = 0;c < imageWidth;c++)
    {

        image[r*imageWidth+c][0] = image[r*imageWidth+c][0]-(alpha*(buffer[r*imageWidth+c][0]-128));
        image[r*imageWidth+c][1] = image[r*imageWidth+c][1]-(alpha*(buffer[r*imageWidth+c][1]-128));
        image[r*imageWidth+c][2] = image[r*imageWidth+c][2]-(alpha*(buffer[r*imageWidth+c][2]-128));
}

}

}

/**************************************************
 TASK 6
**************************************************/

// Display the magnitude and orientation of the edges in an image using the Sobel operator in both X and Y directions
void MainWindow::SobelImage(double** image)
/*
* image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
* NOTE: image is grayscale here, i.e., all 3 channels have the same value which is the grayscale value
*/
{

   int sizeW = 3;
   int sizeH = 3;
   double *kernelx = new double [sizeW*sizeH]{-1,0,1,-2,0,2,-1,0,1};
   //need two new buffers set for sobel x and y pointing to new addresses in memory
   double** bufferx = new double* [imageHeight*imageWidth];
   for (int j = 0; j < imageHeight*imageWidth; j++)
           bufferx[j] = new double[3];
   for (int j =0;j<3;j++){
   for (int i =0;i<imageWidth*imageHeight;i++)

       bufferx[i][j]=image[i][j];

       }
   Convolution(bufferx, kernelx, 3, 3, false);

   double** buffery = new double* [imageHeight*imageWidth];
   for (int j = 0; j < imageHeight*imageWidth; j++)
           buffery[j] = new double[3];
   for (int j =0;j<3;j++){
   for (int i =0;i<imageWidth*imageHeight;i++)

       buffery[i][j]=image[i][j];
   }
   double *kernely = new double [sizeW*sizeH]{1,2,1,0,0,0,-1,-2,-1};
   Convolution(buffery, kernely, 3, 3, false);



   double mag;
   double orien;
//compute Sobel mag and angles
   for(int r = 0;r < imageHeight;r++)
   {
       for(int c = 0;c < imageWidth;c++)
       {           
           mag = sqrt(pow(bufferx[r*imageWidth+c][0]/8,2)+pow(buffery[r*imageWidth+c][0]/8,2));

           orien = atan2((buffery[r*imageWidth+c][0]/8),(bufferx[r*imageWidth+c][0]/8));
           //std::cout <<orien<<endl;
          // std::cout <<"\n"<<endl;
           image[r*imageWidth+c][0] = mag*4.0*((sin(orien) + 1.0)/2.0);
            image[r*imageWidth+c][1] = mag*4.0*((cos(orien) + 1.0)/2.0);
            image[r*imageWidth+c][2] = mag*4.0 - image[r*imageWidth+c][0] - image[r*imageWidth+c][1];
       }
   }
}

/**************************************************
 TASK 7
**************************************************/

// Compute the RGB values at a given point in an image using bilinear interpolation.
void MainWindow::BilinearInterpolation(double** image, double x, double y, double rgb[3])
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * x: x-coordinate (corresponding to columns) of the position whose RGB values are to be found
 * y: y-coordinate (corresponding to rows) of the position whose RGB values are to be found
 * rgb[3]: array where the computed RGB values are to be stored
*/
{

 //need if statements for all oversizing!!!!

//bilinear distances are just nearest neghbors in x and y
    int x_1=floor(x);
    int x_2=x_1+1;
    int y_1=floor(y);
    int y_2=y_1+1;

//check if pixels are on borders or outside of image dimensions set to zero
if(x_1<0 || x_2<0 || x_1>=imageWidth || x_2>=imageWidth || y_1<0 || y_2<0 || y_1>=imageHeight || y_2>=imageHeight){
   for(int i=0;i<3;i++)
       rgb[i]=0;

}

else{
//Find all values at each min distance point from x y. each should be a combo of imageWidth*y_i+x_i
double fq11[3]={image[imageWidth*y_2+x_1][0],image[imageWidth*y_2+x_1][1],image[imageWidth*y_2+x_1][2]};
double fq12[3]={image[imageWidth*y_1+x_1][0],image[imageWidth*y_1+x_1][1],image[imageWidth*y_1+x_1][2]};
double fq22[3]={image[imageWidth*y_1+x_2][0],image[imageWidth*y_1+x_2][1],image[imageWidth*y_1+x_2][2]};
double fq21[3]={image[imageWidth*y_2+x_2][0],image[imageWidth*y_2+x_2][1],image[imageWidth*y_2+x_2][2]};
//compute bilinear interp
for(int i=0;i<3;i++){

rgb[i]=(1/((x_2-x_1)*(y_2-y_1)))*((fq11[i]*(x_2-x)*(y_2-y))+(fq21[i]*(x-x_1)*(y_2-y))+(fq12[i]*(x_2-x)*(y-y_1))+(fq22[i]*(x-x_1)*(y-y_1)));

}
}
}

/*******************************************************************************
 Here is the code provided for rotating an image. 'orien' is in degrees.
********************************************************************************/

// Rotating an image by "orien" degrees
void MainWindow::RotateImage(double** image, double orien)

{
    double radians = -2.0*3.141*orien/360.0;

    // Make a copy of the original image and then re-initialize the original image with 0
    double** buffer = new double* [imageWidth*imageHeight];
    for (int i = 0; i < imageWidth*imageHeight; i++)
    {
        buffer[i] = new double [3];
        for(int j = 0; j < 3; j++)
            buffer[i][j] = image[i][j];
        image[i] = new double [3](); // re-initialize to 0
    }

    for (int r = 0; r < imageHeight; r++)
       for (int c = 0; c < imageWidth; c++)
       {
            // Rotate around the center of the image
            double x0 = (double) (c - imageWidth/2);
            double y0 = (double) (r - imageHeight/2);

            // Rotate using rotation matrix
            double x1 = x0*cos(radians) - y0*sin(radians);
            double y1 = x0*sin(radians) + y0*cos(radians);

            x1 += (double) (imageWidth/2);
            y1 += (double) (imageHeight/2);

            double rgb[3];
            BilinearInterpolation(buffer, x1, y1, rgb);

            // Note: "image[r*imageWidth+c] = rgb" merely copies the head pointers of the arrays, not the values
            image[r*imageWidth+c][0] = rgb[0];
            image[r*imageWidth+c][1] = rgb[1];
            image[r*imageWidth+c][2] = rgb[2];

        }
}

/**************************************************
 TASK 8
**************************************************/

// Find the peaks of the edge responses perpendicular to the edges
void MainWindow::FindPeaksImage(double** image, double thres)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * NOTE: image is grayscale here, i.e., all 3 channels have the same value which is the grayscale value
 * thres: threshold value for magnitude
*/
{

    double e1x;
    double e2x;
    double y1x;
    double y2x;
//First compute Sobel to get mags of image
    int sizeW = 3;
    int sizeH = 3;
    double *kernelx = new double [sizeW*sizeH]{-1,0,1,-2,0,2,-1,0,1};
    //need two new buffers set for sobel x and y pointing to new addresses in memory

    double** bufferx = new double* [imageHeight*imageWidth];
    for (int j = 0; j < imageHeight*imageWidth; j++)
            bufferx[j] = new double[3];
    for (int j =0;j<3;j++){
    for (int i =0;i<imageWidth*imageHeight;i++)

        bufferx[i][j]=image[i][j];

        }
    Convolution(bufferx, kernelx, 3, 3, false);

    double** buffery = new double* [imageHeight*imageWidth];
    for (int j = 0; j < imageHeight*imageWidth; j++)
            buffery[j] = new double[3];
    for (int j =0;j<3;j++){
    for (int i =0;i<imageWidth*imageHeight;i++)

        buffery[i][j]=image[i][j];
    }
    double *kernely = new double [sizeW*sizeH]{1,2,1,0,0,0,-1,-2,-1};
    Convolution(buffery, kernely, 3, 3, false);
//buffer for image copy
    double** buffer = new double* [imageHeight*imageWidth];
    for (int j = 0; j < imageHeight*imageWidth; j++)
            buffer[j] = new double[3];
    for (int j =0;j<3;j++){
    for (int i =0;i<imageWidth*imageHeight;i++)

        buffer[i][j]=image[i][j];

        }

    double mag;

//compute magnitude image
    for(int r = 0;r < imageHeight;r++)
    {
        for(int c = 0;c < imageWidth;c++)
        {

            mag = sqrt(pow(bufferx[r*imageWidth+c][0]/8,2)+pow(buffery[r*imageWidth+c][0]/8,2));
            buffer[r*imageWidth+c][0] = mag;
            buffer[r*imageWidth+c][1] = mag;
            buffer[r*imageWidth+c][2] = mag;
        }
    }
    double rgb1[3];
    double rgb2[3];
    //can use a constant angle of 45 or 135, both do the same
  double radians = -2.0*3.141*45/360.0;
//compute disances and find bilinear interp of angles and mag image
    for(int r = 0;r < imageHeight;r++)
    {
        for(int c = 0;c < imageWidth;c++)
        {

            e1x=c+1*cos(radians);
            e2x=r+1*sin(radians);
            y1x=c-1*cos(radians);
            y2x=r-1*sin(radians);
            BilinearInterpolation(buffer, e1x, y1x, rgb1);
            BilinearInterpolation(buffer, e2x, y2x, rgb2);


            //Check mags and threshold between all new pixels and set values accordingly
            double mag = buffer[r*imageWidth+c][0];
            double mag1 = rgb1[0];
            double mag2 = rgb2[0];

            if ((mag > thres) && (mag > mag1) && (mag > mag2))
                      {

                          image[r*imageWidth+c][0] = 255;
                          image[r*imageWidth+c][1] = 255;
                          image[r*imageWidth+c][2] = 255;
                      }
                      else
                      {
                          image[r*imageWidth+c][0] = 0;
                          image[r*imageWidth+c][1] = 0;
                          image[r*imageWidth+c][2] = 0;
                      }
        }}

}

/**************************************************
 TASK 9 (a)
**************************************************/

// Perform K-means clustering on a color image using random seeds
void MainWindow::RandomSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{
//counter for cluster assignments
int counter[num_clusters];
//create new cluster
double** cluster1=new double*[3];
for (int j = 0; j <3; j++)
        cluster1[j] = new double[num_clusters];
for (int j =0;j<num_clusters;j++){
for (int i =0;i<3;i++)
    cluster1[i][j]=0;
}
//Create and initialize array for means
double** mean=new double*[num_clusters];
for (int j = 0; j <num_clusters; j++)
        mean[j] = new double[3];
for(int i=0;i<num_clusters;i++){
    mean[i][0]=rand() % 256;
    mean[i][1]=rand() % 256;
    mean[i][2]=rand() % 256;
}
//initialzize values
double cluster_R=0;
double cluster_G=0;
double cluster_B=0;
int iteration=0;
int dist=100000;
int epsilon=30;

//while iteration is less than 100 and the distance between pixels and the means are large
while(iteration<100 && dist>epsilon*num_clusters){

/*Loop through all pixels and store the RBG pixels that are closest
 * to the mean in question
 */

    //initialize cluster to zero
    for (int j =0;j<num_clusters;j++){
    for (int i =0;i<3;i++){

        cluster1[i][j]=0;
}
        }
    //initialize counter to 1 to guarantee no division by zero
    for (int j =0;j<num_clusters;j++){
        counter[j]=1;
    }

for (int r = 0; r < imageHeight; r++){
   for (int c = 0; c < imageWidth; c++){
       cluster_R=0;
       cluster_G=0;
       cluster_B=0;
       int index=0;
     double dist=0;
     double old_dist=0;
     //initialize last distance measurement
   old_dist= fabs(image[r*imageWidth+c][0]-mean[0][0])+fabs(image[r*imageWidth+c][1]-mean[0][1])+fabs(image[r*imageWidth+c][2]-mean[0][2]);
   //L1 distance
   for (int j = 1; j < num_clusters; j++){
      cluster_R = fabs(image[r*imageWidth+c][0]-mean[j][0]);
      cluster_G = fabs(image[r*imageWidth+c][1]-mean[j][1]);
      cluster_B = fabs(image[r*imageWidth+c][2]-mean[j][2]);
       dist= cluster_R+ cluster_G+ cluster_B;
//check to see which L1 dist is the smallest
    if(dist<old_dist){
        old_dist=dist;
        //index the cluster
        index=j;
        }
    }
        //Sum all cluster values
       cluster1[0][index]+=image[r*imageWidth+c][0];
       cluster1[1][index]+=image[r*imageWidth+c][1];
       cluster1[2][index]+=image[r*imageWidth+c][2];
       counter[index]+=1;
}

}
//Find new mean values and store them
for(int i=0;i<num_clusters;i++){
for(int k=0;k<3;k++){
mean[i][k]+=cluster1[k][i];
}
//check to see if the clusters were empty
if((mean[i][0]+mean[i][1]+mean[i][2])==0)
{
    //if empty set to last mean
   mean[i][0]=mean[i-1][0];
   mean[i][1]= mean[i-1][1];
   mean[i][2]=mean[i-1][2];
}
else{
mean[i][0]=mean[i][0]/counter[i];
mean[i][1]=mean[i][1]/counter[i];
mean[i][2]=mean[i][2]/counter[i];
}
}

iteration++;
}


/* Loop through and find mean value closest to pixel value,
 * that mean is new pixel value
 */
for (int r = 0; r < imageHeight; r++){
   for (int c = 0; c < imageWidth; c++){
       //Initialize all Values
       cluster_R=0;
       cluster_G=0;
       cluster_B=0;
       int index=0;
       double old_dist=0;
       double dist=0;
       old_dist= fabs(image[r*imageWidth+c][0]-mean[0][0])+fabs(image[r*imageWidth+c][1]-mean[0][1])+fabs(image[r*imageWidth+c][2]-mean[0][2]);

       for (int j = 0; j < num_clusters; j++){

           cluster_R = fabs(image[r*imageWidth+c][0]-mean[j][0]);
           cluster_G = fabs(image[r*imageWidth+c][1]-mean[j][1]);
           cluster_B = fabs(image[r*imageWidth+c][2]-mean[j][2]);
           dist= cluster_R+ cluster_G+ cluster_B;

        if(dist<old_dist){

            old_dist=dist;
            index=j;
        }

}
       //Set each pixel to its closest mean
       image[r*imageWidth+c][0]=mean[index][0];
       image[r*imageWidth+c][1]=mean[index][1];
       image[r*imageWidth+c][2]=mean[index][2];

}

}
}










/**************************************************
 TASK 9 (b)
**************************************************/

// Perform K-means clustering on a color image using seeds from the image itself
void MainWindow::PixelSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{

    int r_num=rand() % imageHeight*imageWidth;

    //counter for cluster assignments
    int counter[num_clusters];
    //create new cluster
    double** cluster1=new double*[3];
    for (int j = 0; j <3; j++)
            cluster1[j] = new double[num_clusters];
    for (int j =0;j<num_clusters;j++){
    for (int i =0;i<3;i++)
        cluster1[i][j]=0;
    }
    //Create and initialize array for means with random pixel value
    double** mean=new double*[num_clusters];
    for (int j = 0; j <num_clusters; j++)
            mean[j] = new double[3];
    for(int i=0;i<num_clusters;i++){
        mean[i][0]=image[r_num][0];
        mean[i][1]=image[r_num][1];
        mean[i][2]=image[r_num][2];
    }

    double cluster_R=0;
    double cluster_G=0;
    double cluster_B=0;
    int iteration=0;
    int dist=100000;
    int epsilon=30;


    while(iteration<100 && dist>epsilon*num_clusters){

    /*Loop through all pixels and store the RBG pixels that are closest
     * to the mean in question
     */

        //initialize cluster to zero
        for (int j =0;j<num_clusters;j++){
        for (int i =0;i<3;i++){

            cluster1[i][j]=0;
    }
            }
        //initialize counter to 1 to guarantee no division by zero
        for (int j =0;j<num_clusters;j++){
            counter[j]=1;
        }

    for (int r = 0; r < imageHeight; r++){
       for (int c = 0; c < imageWidth; c++){
           cluster_R=0;
           cluster_G=0;
           cluster_B=0;
           int index=0;
         double dist=0;
         double old_dist=0;
         //initialize last distance measurement
       old_dist= fabs(image[r*imageWidth+c][0]-mean[0][0])+fabs(image[r*imageWidth+c][1]-mean[0][1])+fabs(image[r*imageWidth+c][2]-mean[0][2]);
           for (int j = 1; j < num_clusters; j++){
          cluster_R = fabs(image[r*imageWidth+c][0]-mean[j][0]);
          cluster_G = fabs(image[r*imageWidth+c][1]-mean[j][1]);
          cluster_B = fabs(image[r*imageWidth+c][2]-mean[j][2]);
           dist= cluster_R+ cluster_G+ cluster_B;
      //if L1 distance is ever less than 100 than break from loop and do not use pixel values
           if(dist<100){
             break;
           }
        if(dist<old_dist){
            old_dist=dist;
            index=j;
            }
        }
            //Sum all cluster values
           cluster1[0][index]+=image[r*imageWidth+c][0];
           cluster1[1][index]+=image[r*imageWidth+c][1];
           cluster1[2][index]+=image[r*imageWidth+c][2];
           counter[index]+=1;
    }

    }
    //Find new mean values and store them
    for(int i=0;i<num_clusters;i++){
    for(int k=0;k<3;k++){
    mean[i][k]+=cluster1[k][i];
    }
    //check to see if the clusters were empty
    if((mean[i][0]+mean[i][1]+mean[i][2])==0)
    {
       mean[i][0]=mean[i-1][0];
       mean[i][1]= mean[i-1][1];
       mean[i][2]=mean[i-1][2];
    }
    else{
    mean[i][0]=mean[i][0]/counter[i];
    mean[i][1]=mean[i][1]/counter[i];
    mean[i][2]=mean[i][2]/counter[i];
    }
    }
    iteration++;
    }


    /* Loop through and find mean value closest to pixel value,
     * that mean is new pixel value
     */
    for (int r = 0; r < imageHeight; r++){
       for (int c = 0; c < imageWidth; c++){
           //Initialize all Values
           cluster_R=0;
           cluster_G=0;
           cluster_B=0;
           int index=0;
           double old_dist=0;
           double dist=0;
           old_dist= fabs(image[r*imageWidth+c][0]-mean[0][0])+fabs(image[r*imageWidth+c][1]-mean[0][1])+fabs(image[r*imageWidth+c][2]-mean[0][2]);

           for (int j = 0; j < num_clusters; j++){

               cluster_R = fabs(image[r*imageWidth+c][0]-mean[j][0]);
               cluster_G = fabs(image[r*imageWidth+c][1]-mean[j][1]);
               cluster_B = fabs(image[r*imageWidth+c][2]-mean[j][2]);
               dist= cluster_R+ cluster_G+ cluster_B;

            if(dist<old_dist){

                old_dist=dist;
                index=j;
            }

    }
           //Set each pixel to its closest mean
           image[r*imageWidth+c][0]=mean[index][0];
           image[r*imageWidth+c][1]=mean[index][1];
           image[r*imageWidth+c][2]=mean[index][2];

    }

    }
}


/**************************************************
 EXTRA CREDIT TASKS
**************************************************/

// Perform K-means clustering on a color image using the color histogram
void MainWindow::HistogramSeedImage(double** image, int num_clusters)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * num_clusters: number of clusters into which the image is to be clustered
*/
{
    // Add your code here
}

// Apply the median filter on a noisy image to remove the noise
void MainWindow::MedianImage(double** image, int radius)
/*
* image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
* radius: radius of the kernel
*/
{
//Note:: Black/white and noise built in functions in the gui are used with the median filter

   // Kernel size from given radius
   int kernelSize = 2*radius + 1;

   // Zero-pad the buffer image based
   int padH = kernelSize/2;
   int padW = kernelSize/2;
   int height = imageHeight + 2*padH;
   int width = imageWidth + 2*padW;

   // Create buffer
   double buffer[height*width];

   // Fill Buffer with border extension
   for (int r = 0; r < height; r++)
   {
       for (int c = 0; c < width; c++)
       {
           // Left
           if (c < padW && r >= padH && r < height - padH)
           {
               buffer[r*width+c] = image[(r-padH)*imageWidth][0];
           }
           // Right
           else if (c >= width - padW && r >= padH && r < height - padH)
           {
               buffer[r*width+c] = image[(r-padH)*imageWidth+(imageWidth-1)][0];
           }
           // Center (Image)
           else if (c >= padW && c < width - padW && r >= padH && r < height - padH)
           {
               buffer[r*width+c] = image[(r - padH)*imageWidth+(c - padW)][0];
           }
       }
   }
   for (int r = 0; r < height; r++)
   {
       for (int c = 0; c < width; c++)
       {
           // Bottom
           if (r >= height - padH)
           {
               buffer[r*width+c] = buffer[(height-padH-1)*width+c];
           }
           // Top
           else if(r < padH)
           {
               buffer[r*width+c] = buffer[(padH)*width+c];
           }
       }
   }

   int count, median;
   // Sliding window of size kernelSize
   double window[kernelSize*kernelSize];

   for(int r = 0;r < imageHeight;r++)
   {
       for(int c = 0;c < imageWidth;c++)
       {
           count = 0;

           // Fill in median window at each index
           for(int r2 = -padH; r2 <= padH; r2++)
           {
               for(int c2 = -padW; c2 <= padW; c2++)
               {
                   window[count] = buffer[(r + r2 + padH)*width + (c + c2 + padW)];
                   count++;
               }
           }

           // Sort window to obtian median
           std::sort(window, window + kernelSize*kernelSize);
           median = window[kernelSize*kernelSize/2];
           image[r*imageWidth+c][0] = median;
           image[r*imageWidth+c][1] = median;
           image[r*imageWidth+c][2] = median;
       }
   }
}

// Apply Bilater filter on an image
void MainWindow::BilateralImage(double **image, double sigmaS, double sigmaI)

{

    int radius = (int)(ceil(3* sigmaS));
    int sizeW = 2*radius +1;
    int sizeH = 2*radius + 1;
    int pad_h = sizeH/2;
    int pad_w = sizeW/2;
    int height = imageHeight + 2*pad_h;
    int width = imageWidth + 2*pad_w;
    int size = (width)*(height);
    //Create a buffer image
    double** buffer = new double* [size];
    for (int j = 0; j < size; j++)
            buffer[j] = new double[3];

    // Compute the kernel to convolve with the image
    double *kernel = new double [sizeW*sizeH];

    for (int r = 0; r < sizeH; r++)
        for (int c = 0; c <  sizeW; c++)
            kernel[r*sizeW + c] = pow(M_E, -((r-radius)*(r-radius) + (c-radius)*(c-radius))/(2*sigmaS*sigmaS));

    // Make sure kernel sums to 1
    NormalizeKernel(kernel, sizeW, sizeH);


    for (int r = 0; r < height; r++)
       {
           for (int c = 0; c < width; c++)
           {
               // Right side of the image
               if ((c >= width - pad_w) && (r >= pad_h) && (r < height - pad_h))
               {
                   buffer[r*width+c][0] = image[(r-pad_h)*imageWidth+(imageWidth-1)][0];
                   buffer[r*width+c][1] = image[(r-pad_h)*imageWidth+(imageWidth-1)][1];
                   buffer[r*width+c][2] = image[(r-pad_h)*imageWidth+(imageWidth-1)][2];
               }
               // Left side of Image
               else if ((c < pad_w) && (r >= pad_h) && (r < height - pad_h))
               {
                   buffer[r*width+c][0] = image[(r-pad_h)*imageWidth][0];
                   buffer[r*width+c][1] = image[(r-pad_h)*imageWidth][1];
                   buffer[r*width+c][2] = image[(r-pad_h)*imageWidth][2];
               }
               // Main Image
               else if ((c >= pad_w) && (c < width - pad_w) && (r >= pad_h) && (r < height - pad_h))
               {
                   buffer[r*width+c][0] = image[(r - pad_h)*imageWidth+(c - pad_w)][0];
                   buffer[r*width+c][1] = image[(r - pad_h)*imageWidth+(c - pad_w)][1];
                   buffer[r*width+c][2] = image[(r - pad_h)*imageWidth+(c - pad_w)][2];
               }
           }
       }
       for (int r = 0; r < height; r++)
       {
           for (int c = 0; c < width; c++)
           {
               // Top rows of the extended image
               if (r < pad_h)
               {
                   buffer[r*width+c][0] = buffer[(pad_h)*width+c][0];
                   buffer[r*width+c][1] = buffer[(pad_h)*width+c][1];
                   buffer[r*width+c][2] = buffer[(pad_h)*width+c][2];
               }
               // Bottom rows of the extended imge
               else if(r >= height - pad_h)
               {
                   buffer[r*width+c][0] = buffer[(height-pad_h-1)*width+c][0];
                   buffer[r*width+c][1] = buffer[(height-pad_h-1)*width+c][1];
                   buffer[r*width+c][2] = buffer[(height-pad_h-1)*width+c][2];
               }
           }
       }

       //For each pixel...
       for (int r = 0; r < imageHeight; r++)
       {
           for (int c = 0; c < imageWidth; c++)
           {

               image[r*imageWidth+c][0] = 0.0;
               image[r*imageWidth+c][1] = 0.0;
               image[r*imageWidth+c][2] = 0.0;


                double totw = 0;
                double r1 = 0;
                double g1 = 0;
                double b1 = 0;
               // Convolve the kernel at each pixel
               for(int rd=-pad_h;rd<= pad_h;rd++)
                   for(int cd=-pad_w;cd<= pad_w;cd++)
                   {

                        // Get the value of the kernel
                       double weight = kernel[(rd + pad_h)*sizeW+ cd + pad_w];
                       double dist = pow((buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][0] - buffer[(r + pad_h)*width + (c + pad_w)][0] ),2) + pow((buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][1] - buffer[(r  + pad_h)*width + (c + pad_w)][1]),2) + pow((buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][2] - buffer[(r + pad_h)*width + (c + pad_w)][2]),2);
                       double weight2 = pow(M_E, -dist/(2*sigmaI*sigmaI));

                       totw += weight*weight2;
                       r1 += weight*weight2*(double) buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][0];
                       g1 += weight*weight2*(double) buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][1];
                       b1 += weight*weight2*(double) buffer[(r + rd + pad_h)*width + (c + cd + pad_w)][2];


              }


               image[r*imageWidth+c][0] = r1/totw;
               image[r*imageWidth+c][1] = g1/totw;
               image[r*imageWidth+c][2] = b1/totw;
           }
    }

}

// Perform the Hough transform
void MainWindow::HoughImage(double** image)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
*/
{
    // Add your code here
}

// Perform smart K-means clustering
void MainWindow::SmartKMeans(double** image)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
*/
{
    // Add your code here
}
