#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace cv;
using namespace std;

class FeedBack {
   VideoCapture cap;
   Point2f calib;
public:
   Point2f getLoc(){
      Point2f avg(0,0);
      Mat frame;
      cap >> frame;
      if(!frame.empty()){
         cv::Mat gray;
         cv::cvtColor(frame,gray,CV_BGR2GRAY);
         vector<Point2f> corners;
         vector<Point2f> fixed;
         
         bool patternfound = cv::findChessboardCorners(gray, Size(6,9), corners,
                                                       CV_CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
         
         if(patternfound){
            cv::cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            for (int i=0;i<corners.size();i++){
               avg += corners[i];
            }
            double avgdev =0 ;
            for (int i=0;i<5;i++){
               for(int j=0;j<8;j++){
                  Point2f diff = corners[i + j*6]-corners[(i+1) + j*6];
                  avgdev += sqrt(diff.x*diff.x + diff.y*diff.y);
                  diff = corners[i + j*6]-corners[i + (j+1)*6];
                  avgdev += sqrt(diff.x*diff.x + diff.y*diff.y);
               }
            }
            
            avgdev /= 5.*8.*2.;//pixels/1.24cm
            //std::cout<<avgdev<<"\n\n";
            avgdev /= 12.4; // this many pixels/inch on CNC
            
            avg.x = avg.x / corners.size();
            avg.y = avg.y / corners.size();
                        
            avg.x = avg.x - (frame.size().width/2.);
            avg.y = avg.y - (frame.size().height/2.);
            
            avg.x = avg.x / avgdev * calib.x;
            avg.y = avg.y / avgdev * calib.y;
            
            cv::drawChessboardCorners(frame, Size(6,9), Mat(corners), patternfound);
            
         }
         rectangle( frame,
                    Point( 0,0),
                    Point(400,50),
                    Scalar( 0, 0, 0 ),
                    -1,
                    8 );
         putText(frame, "CV closeloop CNC Demo.", cvPoint(20,20),
                 FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
         char text[256];
         sprintf(text,"Location: (%f,%f)",avg.x,avg.y);
         putText(frame, text, cvPoint(20,40),
                 FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
         
         imshow("image",frame);
      }
      
         if(waitKey(30) >= 0) exit(0);
      return avg;
   }
   FeedBack(Point2f x,int v):calib(x){
      cap.open(v);
      if(!cap.isOpened()){  // check if we succeeded
         printf("No camera!");
         exit(0);
      }
      cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
      cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
      
      namedWindow("image",-1);
      
   }
};

class Drive{
   int tty;
   int zstate;
   int serialSetup(){
      int tty_fd;
      struct termios tio;
      memset(&tio,0,sizeof(tio));
      tio.c_iflag=0;
      tio.c_oflag=0;
      tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
      tio.c_lflag=0;
      tio.c_cc[VMIN]=1;
      tio.c_cc[VTIME]=255;
      
      tty_fd=open("/dev/tty.usbserial-A800crpp", O_RDWR | O_NONBLOCK);
      cfsetospeed(&tio,B9600);            // 9600 baud
      cfsetispeed(&tio,B9600);            // 9600 baud
      
      tcsetattr(tty_fd,TCSANOW,&tio);
      return tty_fd;
   }
public:
   Drive(){
      tty = serialSetup();
      if(tty == -1){
         printf("failed to open port\n");
         exit(0);
      }
      zstate = 5;
   }
   void move(Point3f dir){
      char c[2];
      c[0] = 'k';
      c[1] = 128;
      if(dir.z > 0 && zstate < 5){
         if(zstate > 3){
            c[0] = 'k';
            c[1] = 128;
            
            write(tty,&c,2);
            usleep(100000);
         }else{
            c[0] = 'r';
            c[1] = 150;
         }
         zstate ++;
      }else if(dir.z < 0 && zstate > 0){
         if(zstate < 2){
            c[0] = 'b';
            c[1] = 128;
            write(tty,&c,2);
            usleep(100000);
         }else{
            c[0] = 'f';
            c[1] = 255;
         }
         zstate --;
      }else{
         if(abs(dir.x) > abs(dir.y)){
            if(dir.x > 0){
               c[0] = 'a';
               c[1] = 255 * (dir.x);
            }else{
               c[0] = 'd';
               c[1] = 255 * (-dir.x);
            }
         }else{
            if(dir.y > 0){
               c[0] = 'w';
               c[1] = 255 * (dir.y);
            }else{
               c[0] = 's';
               c[1] = 255 * (-dir.y);
            }
         }
      }
      write(tty,&c,2);
      
      //if(read(tty,&c,1))
         //write(STDOUT_FILENO,&c,1);
   }
};

class Gparse{
   ifstream in;
   double x,y,z;
public:
   Gparse(char* fn){
      in.open(fn,ios::in);
      if(!in.is_open()){
         cout << "Can't open file";
         exit(0);
      }
      x=y=z=0;
   }
   Point3f getLoc(){
      return Point3f(x,y,z);
   }
   int read(){
      int output = 0;
      string line;
      if(in.eof())
         return -1;
      getline(in,line);
      output = line.length();
      int prevs = 0;
      int state = 0;
      for(int i=0;i<output;i++){
         if(line[i] == ' ' || i == (output-1)){
            if(state == 0){
               if(line[prevs] == 'G'){
                  switch(line[prevs + 2]){
                     case '0':
                        state = 1;
                        break;
                     case '1':
                        state = 2;
                        break;
                  }
               }
            }else if(state > 0){
               double val;
               val = atof(line.substr(prevs+1,(i-(prevs+1))).c_str());
               switch(line[prevs]){
                  case 'X':
                     x = val;
                     break;
                  case 'Y':
                     y = val;
                     break;
                  case 'Z':
                     z = val;
                     break;
               }
               
            }
            prevs = i + 1;
         }
      }
      //cout << state << " " << x << " " << y << " " << z << "\n";
      return output;
   };
};

int main(int argc,char** argv)
{
   
   FeedBack fb(Point2f(1,1),0);
   Drive dv;
   Gparse in(argv[1]);
   double speed = 0.3;
   
   Point3f to(0,0,1);
   Point3f from;
   
   while(in.read()>0){
      from = to;
      to = in.getLoc();
      
      double dl = sqrt((to.x-from.x)*(to.x-from.x) + (to.y-from.y)*(to.y-from.y));
      double dx = (to.x - from.x)/(dl/speed);
      double dy = (to.y - from.y)/(dl/speed);
      double dz = to.z - from.z;
      
      Point3f movedir(0,0,0);
      
      if( abs(dz) > 0){
         movedir.z = to.z;
         for(int k = 0; k < 10 ;k ++){
            dv.move(movedir);
         }
      }else{
         for (int i = 0 ; i< dl/speed; i++){
            double div = 0;
            Point2f newL(from.x + i*dx,from.y + i*dy);
            do{
               Point2f f = fb.getLoc();
               //if(f.x == 0 && f.y == 0) continue;
               div = (f.x-newL.x) * (f.x-newL.x) + (f.y-newL.y) * (f.y-newL.y);
               movedir.x = (f.x - newL.x)/2;
               movedir.y = (f.y - newL.y)/2;
               
               if(abs(movedir.x)> 1) movedir.x = 1 * (movedir.x / abs(movedir.x));
               if(abs(movedir.x)< 0.05 && abs((f.x-newL.x)) > 0.01) movedir.x = 0.05 * (movedir.x / abs(movedir.x));
               
               if(abs(movedir.y)> 1) movedir.y = 1 * (movedir.y / abs(movedir.y));
               if(abs(movedir.y)< 0.05 && abs((f.y-newL.y)) > 0.01) movedir.y = 0.05 * (movedir.y / abs(movedir.y));
               
               dv.move(movedir);
               //cout << newL << "|||" << f << "\n";
               
            }while(div > 0.01);
         }
      }
   }
   return EXIT_SUCCESS;
}
