#include "mainwindow.h"
#include "math.h"
#include "ui_mainwindow.h"
#include <QtGui>
#include "Matrix.h"
#include <iostream>
#include <stdlib.h>


/*******************************************************************************
Draw detected Harris corners
    cornerPts - corner points
    numCornerPts - number of corner points
    imageDisplay - image used for drawing

    Draws a red cross on top of detected corners
*******************************************************************************/
void MainWindow::DrawCornerPoints(CIntPt *cornerPts, int numCornerPts, QImage &imageDisplay)
{
   int i;
   int r, c, rd, cd;
   int w = imageDisplay.width();
   int h = imageDisplay.height();

   for(i=0;i<numCornerPts;i++)
   {
       c = (int) cornerPts[i].m_X;
       r = (int) cornerPts[i].m_Y;

       for(rd=-2;rd<=2;rd++)
           if(r+rd >= 0 && r+rd < h && c >= 0 && c < w)
               imageDisplay.setPixel(c, r + rd, qRgb(255, 0, 0));

       for(cd=-2;cd<=2;cd++)
           if(r >= 0 && r < h && c + cd >= 0 && c + cd < w)
               imageDisplay.setPixel(c + cd, r, qRgb(255, 0, 0));
   }
}

/*******************************************************************************
Compute corner point descriptors
    image - input image
    cornerPts - array of corner points
    numCornerPts - number of corner points

    If the descriptor cannot be computed, i.e. it's too close to the boundary of
    the image, its descriptor length will be set to 0.

    
*******************************************************************************/
void MainWindow::ComputeDescriptors(QImage image, CIntPt *cornerPts, int numCornerPts)
{
    int r, c, cd, rd, i, j;
    int w = image.width();
    int h = image.height();
    double *buffer = new double [w*h];
    QRgb pixel;

    // Descriptor parameters
    double sigma = 2.0;
    int rad = 4;

    // Computer descriptors from green channel
    for(r=0;r<h;r++)
       for(c=0;c<w;c++)
        {
            pixel = image.pixel(c, r);
            buffer[r*w + c] = (double) qGreen(pixel);
        }

    // Blur
    GaussianBlurImage(buffer, w, h, sigma);

    // Compute the desciptor from the difference between the point sampled at its center
    // and eight points sampled around it.
    for(i=0;i<numCornerPts;i++)
    {
        int c = (int) cornerPts[i].m_X;
        int r = (int) cornerPts[i].m_Y;

        if(c >= rad && c < w - rad && r >= rad && r < h - rad)
        {
            double centerValue = buffer[(r)*w + c];
            int j = 0;

            for(rd=-1;rd<=1;rd++)
                for(cd=-1;cd<=1;cd++)
                    if(rd != 0 || cd != 0)
                {
                    cornerPts[i].m_Desc[j] = buffer[(r + rd*rad)*w + c + cd*rad] - centerValue;
                    j++;
                }

            cornerPts[i].m_DescSize = DESC_SIZE;
        }
        else
        {
            cornerPts[i].m_DescSize = 0;
        }
    }

    delete [] buffer;
}

/*******************************************************************************
Draw matches between images
    matches - matching points
    numMatches - number of matching points
    image1Display - image to draw matches
    image2Display - image to draw matches

    Draws a green line between matches
*******************************************************************************/
void MainWindow::DrawMatches(CMatches *matches, int numMatches, QImage &image1Display, QImage &image2Display)
{
    int i;
    // Show matches on image
    QPainter painter;
    painter.begin(&image1Display);
    QColor green(0, 250, 0);
    QColor red(250, 0, 0);

    for(i=0;i<numMatches;i++)
    {
        painter.setPen(green);
        painter.drawLine((int) matches[i].m_X1, (int) matches[i].m_Y1, (int) matches[i].m_X2, (int) matches[i].m_Y2);
        painter.setPen(red);
        painter.drawEllipse((int) matches[i].m_X1-1, (int) matches[i].m_Y1-1, 3, 3);
    }

    QPainter painter2;
    painter2.begin(&image2Display);
    painter2.setPen(green);

    for(i=0;i<numMatches;i++)
    {
        painter2.setPen(green);
        painter2.drawLine((int) matches[i].m_X1, (int) matches[i].m_Y1, (int) matches[i].m_X2, (int) matches[i].m_Y2);
        painter2.setPen(red);
        painter2.drawEllipse((int) matches[i].m_X2-1, (int) matches[i].m_Y2-1, 3, 3);
    }

}


/*******************************************************************************
Given a set of matches computes the "best fitting" homography
    matches - matching points
    numMatches - number of matching points
    h - returned homography
    isForward - direction of the projection (true = image1 -> image2, false = image2 -> image1)
*******************************************************************************/
bool MainWindow::ComputeHomography(CMatches *matches, int numMatches, double h[3][3], bool isForward)
{
    int error;
    int nEq=numMatches*2;

    dmat M=newdmat(0,nEq,0,7,&error);
    dmat a=newdmat(0,7,0,0,&error);
    dmat b=newdmat(0,nEq,0,0,&error);

    double x0, y0, x1, y1;

    for (int i=0;i<nEq/2;i++)
    {
        if(isForward == false)
        {
            x0 = matches[i].m_X1;
            y0 = matches[i].m_Y1;
            x1 = matches[i].m_X2;
            y1 = matches[i].m_Y2;
        }
        else
        {
            x0 = matches[i].m_X2;
            y0 = matches[i].m_Y2;
            x1 = matches[i].m_X1;
            y1 = matches[i].m_Y1;
        }


        //Eq 1 for corrpoint
        M.el[i*2][0]=x1;
        M.el[i*2][1]=y1;
        M.el[i*2][2]=1;
        M.el[i*2][3]=0;
        M.el[i*2][4]=0;
        M.el[i*2][5]=0;
        M.el[i*2][6]=(x1*x0*-1);
        M.el[i*2][7]=(y1*x0*-1);

        b.el[i*2][0]=x0;
        //Eq 2 for corrpoint
        M.el[i*2+1][0]=0;
        M.el[i*2+1][1]=0;
        M.el[i*2+1][2]=0;
        M.el[i*2+1][3]=x1;
        M.el[i*2+1][4]=y1;
        M.el[i*2+1][5]=1;
        M.el[i*2+1][6]=(x1*y0*-1);
        M.el[i*2+1][7]=(y1*y0*-1);

        b.el[i*2+1][0]=y0;

    }
    int ret=solve_system (M,a,b);
    if (ret!=0)
    {
        freemat(M);
        freemat(a);
        freemat(b);

        return false;
    }
    else
    {
        h[0][0]= a.el[0][0];
        h[0][1]= a.el[1][0];
        h[0][2]= a.el[2][0];

        h[1][0]= a.el[3][0];
        h[1][1]= a.el[4][0];
        h[1][2]= a.el[5][0];

        h[2][0]= a.el[6][0];
        h[2][1]= a.el[7][0];
        h[2][2]= 1;
    }

    freemat(M);
    freemat(a);
    freemat(b);

    return true;
}





Blur a single channel floating point image with a Gaussian.
    image - input and output image
    w - image width
    h - image height
    sigma - standard deviation of Gaussian

    This code should be very similar to the code you wrote for assignment 1.
*******************************************************************************/
// Convolve the image with the kernel
void MainWindow::Convolution(double* image,int imageWidth,int imageHeight,  double *kernel, int kernelWidth, int kernelHeight, bool add)
/*
 * image: input image in matrix form of size (imageWidth*imageHeight)*3 having double values
 * kernel: 1-D array of kernel values
 * kernelWidth: width of the kernel
 * kernelHeight: height of the kernel

*/
{
    //Zero-pad the buffer image based on the kernel height and width
       int edge_h = kernelHeight/2;
       int edge_w = kernelWidth/2;
       int height = imageHeight + 2*edge_h;
       int width = imageWidth + 2*edge_w;
       int size = (width)*(height);
       //Create a buffer image
       double buffer[size];


          for (int r = 0; r < height; r++)
          {
              for (int c = 0; c < width; c++)
              {
                  // Left
                  if (c <  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c] = image[(r-edge_h)*imageWidth];

                  }
                  // Right
                  else if (c >= width -  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c] = image[(r-edge_h)*imageWidth+(imageWidth-1)];

                  }
                  // Center (Image)
                  else if (c >=  edge_w && c < width -  edge_w && r >= edge_h && r < height - edge_h)
                  {
                      buffer[r*width+c] = image[(r - edge_h)*imageWidth+(c -  edge_w)];

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
                      buffer[r*width+c] = buffer[(height-edge_h-1)*width+c];

                  }
                  // Top
                  else if(r < edge_h)
                  {
                      buffer[r*width+c] = buffer[(edge_h)*width+c];

                  }
              }
          }


       // For each pixel in the image...
       for(int r = 0;r < imageHeight;r++)
       {
           for(int c = 0;c < imageWidth;c++)
           {
               image[r*imageWidth+c] = 0.0;

               // Convolve the kernel at each pixel
               for(int rd=-edge_h;rd<= edge_h;rd++)
                   for(int cd=-edge_w;cd<= edge_w;cd++)
                   {

                        // Get the value of the kernel
                       double weight = kernel[(rd + edge_h)*kernelWidth + cd + edge_w];

                       image[r*imageWidth+c] += weight*(double) buffer[(r + rd + edge_h)*width + (c + cd + edge_w)];

                   }
           }
       }
if(add==true){
  for(int j=0;j<imageWidth*imageHeight;j++){

  image[j]=image[j]+128;

  }



   }
}






void MainWindow::GaussianBlurImage(double *image, int w, int h, double sigma)
{
    int radius = (int)(ceil(3* sigma));
    int sizeW = 2*radius +1;
    int sizeH = 2*radius + 1;
    // Compute the kernel to convolve with the image
    double *kernel = new double [sizeW]; //vector form

    for (int r = 0; r < sizeW; r++)
    {
         for (int c = 0; c <  1; c++){
       //Gaussian for seperate rows
    kernel[r] = (1/(sqrt(2*M_PI)*sigma))*pow(M_E, -((r-radius)*(r-radius))/(2*sigma*sigma));
}}
    // Make sure kernel sums to 1
   NormalizeKernel(kernel, sizeW, 1);
    Convolution(image,w,h, kernel, sizeW,1, false);


 for (int r = 0; r < 1; r++){
        for (int c = 0; c <  sizeH; c++){
            //Gaussian for Columns
     kernel[c] = (1/(sqrt(2*M_PI)*sigma))*pow(M_E, -((c-radius)*(c-radius))/(2*sigma*sigma));
}}
    // Make sure kernel sums to 1
    NormalizeKernel(kernel, 1, sizeH);
    Convolution(image,w,h,kernel, 1, sizeH, false);
    // To access the pixel (c,r), use image[r*width + c].

}

void MainWindow::NormalizeKernel(double *kernel, int kernelWidth, int kernelHeight)
{
    double denom = 0.000001; int i;
    for(i = 0; i < kernelWidth*kernelHeight; i++)
    {
        denom += kernel[i];
    }
    for(i = 0; i < kernelWidth*kernelHeight; i++)
    {
        kernel[i] /= denom;
    }
}


/*******************************************************************************
Detect Harris corners.
    image - input image
    sigma - standard deviation of Gaussian used to blur corner detector
    thres - Threshold for detecting corners
    cornerPts - returned corner points
    numCornerPts - number of corner points returned
    imageDisplay - image returned to display (for debugging)
*******************************************************************************/
void MainWindow::HarrisCornerDetector(QImage image, double sigma, double thres, CIntPt **cornerPts, int &numCornerPts, QImage &imageDisplay)
{
    int r, c;
    int w = image.width();
    int h = image.height();
    double *buffer = new double [w*h];
     double *x = new double [w*h];
      double *y = new double [w*h];
      double *xx = new double [w*h];
       double *yy = new double [w*h];
        double *xy = new double [w*h];
    QRgb pixel;

    numCornerPts = 0;

    // Compute the corner response using just the green channel
    for(r=0;r<h;r++)
       for(c=0;c<w;c++)
        {
            pixel = image.pixel(c,r);

            buffer[r*w + c] = (double) qGreen(pixel);
        }

    for(int i = 0; i < w*h; i++)
    {

    x[i]=buffer[i];
    y[i]=buffer[i];
    }

    // Compute the kernel to convolve with the image
    double *kernel = new double [3]{-1,0,1}; //vector form
//get derivatives
    Convolution(x,w,h,kernel, 3, 1, false);
    Convolution(y,w,h,kernel, 1, 3, false);


        for(int i = 0; i < w*h; i++)
        {

    xx[i]=x[i]*x[i];
    yy[i]=y[i]*y[i];
    xy[i]=x[i]*y[i];

}
        GaussianBlurImage(xx, w, h,sigma);
        GaussianBlurImage(yy, w, h,sigma);
        GaussianBlurImage(xy, w, h,sigma);
        // Compute corner response at each pixel
           double det, trace;
           double *R = new double [w*h];

           for (int i = 0; i < w*h; i++)
           {
               det = (xx[i] * yy[i]) - pow(xy[i],2);
               trace = xx[i] + yy[i];
               R[i] = det / trace;
           }

            //Find Peaks look at 8 surrounding pixels
           bool *p_index = new bool [w*h];
           for (int r = 0; r < h; r++)
           {
               for (int c = 0; c < w; c++)
               {
                    p_index[r*w+c] = 0;
               }
           }

     //   Compute the number of corner points
        int r_p, c_p;
        for (int r = 0; r < h; r++)
        {
            for (int c = 0; c < w; c++)
            {
                //look at neighbors and Harris response
                bool find_peak = (R[r*w+c] > thres);
                for (int i = -1; (i <= 1 && find_peak); i++)
                {
                    for (int j = -1; (j <= 1 && find_peak); j++)
                    {
                        r_p = r + i;
                        c_p = c + j;

                        // Check to see if peak is of its nearest neghbors
                        if (0 <= r_p && r_p < h && 0 <= c_p && c_p < w && (c_p != c || r_p != r))
                        {
                            if (R[r*w+c] <= R[r_p*w+c_p])
                            {
                                find_peak = 0;
                            }
                        }
                    }
                }
                if (find_peak)
                {
                    p_index[r*w+c] =1;
                    numCornerPts++;
                }
            }
        }



           *cornerPts = new CIntPt [numCornerPts];
        int ind = 0;
            for (int r = 0; r < h; r++)
            {
                for (int c = 0; c < w; c++)
                {
                    if(p_index[r*w+c]){
                        (*cornerPts)[ind].m_X = c;
                        (*cornerPts)[ind].m_Y = r;
                        ind++;
                    }
                }
            }


        // Obtain max value for normalization
            double max = 0;
            for (int r = 0; r < h; r++)
            {
                for (int c = 0; c < w; c++)
                {
                    if (R[r*w+c] > max)
                    {
                        max = R[r*w+c];
                    }
                }
            }


            // Normalize and dsiplay Harris Response
//                for (int r = 0; r < h; r++)
//                {
//                    for (int c = 0; c < w; c++)
//                    {
//                       int pixel = (int)((R[r*w+c]/max)*255);
//                        imageDisplay.setPixel(c, r, qRgb(pixel, pixel, pixel));
//                    }
//                }



    DrawCornerPoints(*cornerPts, numCornerPts, imageDisplay);

    delete [] buffer;


}





/*******************************************************************************
Find matching corner points between images.
    image1 - first input image
    cornerPts1 - corner points corresponding to image 1
    numCornerPts1 - number of corner points in image 1
    image2 - second input image
    cornerPts2 - corner points corresponding to image 2
    numCornerPts2 - number of corner points in image 2
    matches - set of matching points to be returned
    numMatches - number of matching points returned
    image1Display - image used to display matches
    image2Display - image used to display matches
*******************************************************************************/
void MainWindow::MatchCornerPoints(QImage image1, CIntPt *cornerPts1, int numCornerPts1,
                             QImage image2, CIntPt *cornerPts2, int numCornerPts2,
                            CMatches **matches, int &numMatches, QImage &image1Display, QImage &image2Display)

{
    numMatches = 0;

    // Compute the descriptors for each corner point.
   
    // If cornerPts1[i].m_DescSize = 0, it was not able to compute a descriptor for that point
    ComputeDescriptors(image1, cornerPts1, numCornerPts1);
    ComputeDescriptors(image2, cornerPts2, numCornerPts2);


//initialize distances
int L1=0;
int old_L1;
int index=0;

//Initiate Memory Allocation
double *bmx  = (double *) malloc(numMatches * sizeof(double));
double *bmy  = (double *) malloc(numMatches * sizeof(double));
double *bm2x  = (double *) malloc(numMatches * sizeof(double));
double *bm2y  = (double *) malloc(numMatches * sizeof(double));

for(int k=1;k<numCornerPts2;k++){

    if(cornerPts2[k].m_DescSize != 0){
        old_L1=0;
      //initialize L1 distance
    for(int n=0;n<8;n++){
       old_L1 += fabs(cornerPts1[0].m_Desc[n]-cornerPts2[k].m_Desc[n]);
    }

    for(int i=0;i<numCornerPts1;i++){

        if(cornerPts1[i].m_DescSize != 0){
            L1=0;
            for(int j=0;j<DESC_SIZE;j++){

                 L1+=fabs(cornerPts1[i].m_Desc[j]-cornerPts2[k].m_Desc[j]);
            }
                 if(L1<old_L1){
                    index=i;
                     old_L1=L1;

                 }

        }

    }
bmx[numMatches]=cornerPts1[index].m_X;
bmy[numMatches]=cornerPts1[index].m_Y;
bm2x[numMatches]=cornerPts2[k].m_X;
bm2y[numMatches]=cornerPts2[k].m_Y;
numMatches++;
//Reallocate memory dynamically to update for numMatches
double *temp=(double *)realloc(bmx,(numMatches+1)*sizeof(double));
double *temp1=(double *)realloc(bmy,(numMatches+1)*sizeof(double));
double *temp2=(double *)realloc(bm2x,(numMatches+1)*sizeof(double));
double *temp3=(double *)realloc(bm2y,(numMatches+1)*sizeof(double));
bmx=temp;
bmy=temp1;
bm2x=temp2;
bm2y=temp3;
    }
}
std::cout<<numMatches<<"\n";
*matches = new CMatches [numMatches];
for (int r = 0; r <numMatches; r++)
{
    (*matches)[r].m_X1 = bmx[r];
    (*matches)[r].m_Y1 = bmy[r];
    (*matches)[r].m_X2 =bm2x[r];
    (*matches)[r].m_Y2 = bm2y[r];

}

    DrawMatches(*matches, numMatches, image1Display, image2Display);
}





/*******************************************************************************
Project a point (x1, y1) using the homography transformation h
    (x1, y1) - input point
    (x2, y2) - returned point
    h - input homography used to project point
*******************************************************************************/
void MainWindow::Project(double x1, double y1, double &x2, double &y2, double h[3][3])
{
    /*

void Project(double x1, double y1, doube &x2, double &y2, double h[3][3])
This should project point (x1, y1) using the homography h.
Return the projected point (x2, y2). See the slides for details on how to project using homogeneous coordinates.
*/
x2=(h[0][0]*x1+h[0][1]*y1+h[0][2])/(h[2][0]*x1+h[2][1]*y1+h[2][2]);
y2=(h[1][0]*x1+h[1][1]*y1+h[1][2])/(h[2][0]*x1+h[2][1]*y1+h[2][2]);


}

/*******************************************************************************
Count the number of inliers given a homography.  This is a helper function for RANSAC.
    h - input homography used to project points (image1 -> image2
    matches - array of matching points
    numMatches - number of matchs in the array
    inlierThreshold - maximum distance between points that are considered to be inliers

    Returns the total number of inliers.
*******************************************************************************/
int MainWindow::ComputeInlierCount(double h[3][3], CMatches *matches, int numMatches, double inlierThreshold)
{


 
    // Add your code here.
    int numin=0;
    double x1;
    double y1;
    double x2;
    double y2;
    double prjx2;
    double prjy2;
    double dist;


    for(int i=0;i<numMatches;i++){

      x1 = matches[i].m_X1;
      y1 = matches[i].m_Y1;
      x2 = matches[i].m_X2;
      y2 = matches[i].m_Y2;
    Project(x1,y1,prjx2,prjy2,h);

      dist = sqrt(pow(prjx2-x2,2) + pow(prjy2-y2,2));
if (dist < inlierThreshold){

        numin++;
    }

    }

    return numin;
}


/*******************************************************************************
Compute homography transformation between images using RANSAC.
    matches - set of matching points between images
    numMatches - number of matching points
    numIterations - number of iterations to run RANSAC
    inlierThreshold - maximum distance between points that are considered to be inliers
    hom - returned homography transformation (image1 -> image2)
    homInv - returned inverse homography transformation (image2 -> image1)
    image1Display - image used to display matches
    image2Display - image used to display matches
*******************************************************************************/
void MainWindow::RANSAC(CMatches *matches, int numMatches, int numIterations, double inlierThreshold,
                        double hom[3][3], double homInv[3][3], QImage &image1Display, QImage &image2Display)
{
   
    /*
This function takes a list of potentially matching points between two images and returns
the homography transformation that relates them.


     */

    int randMatch[4];
       int inlierCount;
       int maxInlier = 0;
       double new_h[3][3];

       for (int i = 0; i < numIterations; i++)
       {
           for (int j = 0; j < 4; j++)
           {
               randMatch[j] = rand() % numMatches;
               //continously check to make sure rand matches is unique points
               for (int k = 0; k < j; k++)
               {
                   if(randMatch[k] == randMatch[j])
                   {
                       j = 0;
                       break;
                   }
               }
           }
                //fill matches buffer with random indices
           CMatches Matches_buff[4];
           Matches_buff[0] = matches[randMatch[0]];
           Matches_buff[1] = matches[randMatch[1]];
           Matches_buff[2] = matches[randMatch[2]];
           Matches_buff[3] = matches[randMatch[3]];

           double h[3][3];
           ComputeHomography(Matches_buff, 4, h, true);

           inlierCount = ComputeInlierCount(h, matches, numMatches, inlierThreshold);
                //condition for finding max inlier
           if (inlierCount > maxInlier)
           {
               maxInlier = inlierCount;
               // Set optimally computed homography
               for(int j = 0; j < 3; j++)
               {
                   for(int k = 0; k < 3; k++)
                   {
                       new_h[j][k] = h[j][k];
                   }
               }
           }
       }

       // Compute homography using best inliers
       CMatches *inliers = new CMatches[maxInlier];
       int numInliers = 0;
       for (int i = 0; i < numMatches; i++)
       {
           double x2Proj;
           double y2Proj;
           Project(matches[i].m_X1, matches[i].m_Y1, x2Proj, y2Proj,new_h);
           double distance = sqrt(pow(matches[i].m_X2-x2Proj, 2) + pow(matches[i].m_Y2-y2Proj, 2));
           if (distance < inlierThreshold)
           {
               inliers[numInliers] = matches[i];
               numInliers++;
           }
       }

       ComputeHomography(inliers, numInliers, hom, true);
       ComputeHomography(inliers, numInliers, homInv, false);

       // After you're done computing the inliers, display the corresponding matches.
       DrawMatches(inliers, numInliers, image1Display, image2Display);
   }


/*******************************************************************************
Bilinearly interpolate image (helper function for Stitch)
    image - input image
    (x, y) - location to interpolate
    rgb - returned color values

 
*******************************************************************************/
bool MainWindow::BilinearInterpolation(QImage *image, double x, double y, double rgb[3])

{

//bilinear distances are just nearest neghbors in x and y
int imageWidth = image->width();
int imageHeight = image->height();
    int x_1=floor(x);
    int x_2=x_1+1;
    int y_1=floor(y);
    int y_2=y_1+1;
    QRgb pixel11 = image->pixel(x_1, y_1);
    QRgb pixel12 = image->pixel(x_1, y_2);
    QRgb pixel21 = image->pixel(x_2, y_1);
    QRgb pixel22 = image->pixel(x_2, y_2);

//check if pixels are on borders or outside of image dimensions set to zero
if(x_1<0 || x_2<0 || x_1>=imageWidth || x_2>=imageWidth || y_1<0 || y_2<0 || y_1>=imageHeight || y_2>=imageHeight){
   for(int i=0;i<3;i++)
       rgb[i]=0;
    return false;
}

else{
//Find all values at each min distance point from x y. each should be a combo of imageWidth*y_i+x_i
    rgb[0] = (1/((x_2-x_1)*(y_2-y_1)))*((qRed(pixel12)*(x_2-x)*(y_2-y))+(qRed(pixel22)*(x-x_1)*(y_2-y))+(qRed(pixel11)*(x_2-x)*(y-y_1))+(qRed(pixel21)*(x-x_1)*(y-y_1)));
    rgb[1] = (1/((x_2-x_1)*(y_2-y_1)))*((qGreen(pixel12)*(x_2-x)*(y_2-y))+(qGreen(pixel22)*(x-x_1)*(y_2-y))+(qGreen(pixel11)*(x_2-x)*(y-y_1))+(qGreen(pixel21)*(x-x_1)*(y-y_1)));
    rgb[2] = (1/((x_2-x_1)*(y_2-y_1)))*((qBlue(pixel12)*(x_2-x)*(y_2-y))+(qBlue(pixel22)*(x-x_1)*(y_2-y))+(qBlue(pixel11)*(x_2-x)*(y-y_1))+(qBlue(pixel21)*(x-x_1)*(y-y_1)));
return true;
}
}
/*******************************************************************************
Stitch together two images using the homography transformation
    image1 - first input image
    image2 - second input image
    hom - homography transformation (image1 -> image2)
    homInv - inverse homography transformation (image2 -> image1)
    stitchedImage - returned stitched image
*******************************************************************************/
void MainWindow::Stitch(QImage image1, QImage image2, double hom[3][3], double homInv[3][3], QImage &stitchedImage)
{
    // Width and height of stitchedImage
    int ws = 0;
    int hs = 0;

    // variables for width and heights
    int image2Width = image2.width();
    int image2Height = image2.height();
    int image1Width = image1.width();
    int image1Height = image1.height();

    // need to projects all corners from inage 2 to image 1
    double ULX;
    double ULY;
    Project(0, 0, ULX, ULY, homInv);
    double URX;
    double URY;
    Project(image2Width-1, 0, URX, URY, homInv);
    double BLX;
    double BLY;
    Project(0, image2Height-1, BLX, BLY, homInv);
    double BRX;
    double BRY;
    Project(image2Width-1, image2Height-1, BRX, BRY, homInv);

    // Calculate the width and height of stitchedImage, imgae projections could be anywhere on image 1, so need min and max values
    double minWidth = min(min(ULX, URX), min(BRX, BLX));
    double minHeight = min(min(ULY, URY), min(BRY, BLY));
    double maxWidth = max(max(ULX, URX), max(BRX, BLX));
    double maxHeight = max(max(ULY, URY), max(BRY, BLY));

    // make positive
    int x_off_set = abs(min(0, (int)floor(minWidth)));
    int y_off_set = abs(min(0, (int)floor(minHeight)));

    // new stitched image width and height
    ws = x_off_set + max(image1Width, (int)ceil(maxWidth));
    hs = y_off_set + max(image1Height, (int)ceil(maxHeight));

    // Allocate stichted image
    stitchedImage = QImage(ws, hs, QImage::Format_RGB32);
    stitchedImage.fill(qRgb(0,0,0));

    // correctly place first image based on offset
    for (int r = 0; r < image1Height; r++)
    {
        for (int c = 0; c < image1Width; c++)
        {
            stitchedImage.setPixel(x_off_set + c, y_off_set + r, image1.pixel(c, r));
        }
    }

    // based on offset go through and fill stitched image with correctly placed pixels
    for (int r = 0; r < hs; r++)
    {
        for (int c = 0; c < ws; c++)
        {
            // Project
            double prjx, prjy;
            Project(c-x_off_set, r-y_off_set, prjx, prjy, hom);

            if (0 <= prjx && prjx < image2Width && 0 <= prjy && prjy < image2Height)
            {
                double rgb[3];
                BilinearInterpolation(&image2, prjx, prjy, rgb);
                stitchedImage.setPixel(c, r, qRgb((int)(floor(rgb[0]+0.5)),(int)(floor(rgb[1]+0.5)),(int)(floor(rgb[2]+0.5))));
            }
        }
    }
}

