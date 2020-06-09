#include "robot.hpp"
#include <cmath>
int N = 3; //number of pixel colours
/** Compresses the image down into 4 colours
 *  White, Green, Yellow, Red as these are the
 *  most common colours in the maze
*/
double** loadImageToMatrix(ImagePPM& image){
  int size = image.height*image.width;
  double** X = new double*[size]; //create a matrix X to act as an image we can edit
  for(int i = 0; i < size; i++){
    X[i] = new double[N];
  }
  std::cout<<"X initialised"<<std::endl;
  for ( int row = 0 ; row < image.height; row++) {
    for ( int column = 0; column < image.width ; column++) {
      double redPix = get_pixel(image, row, column, 0);
      double greenPix = get_pixel(image, row, column, 1);
      double bluePix = get_pixel(image, row, column, 2);
      X[image.width*row+column][0] = (int)redPix;
      X[image.width*row+column][1] = (int)greenPix;
      X[image.width*row+column][2] = (int)bluePix;
    }

  }
  return X;

}

void compressImage(ImagePPM& image){
  std::cout<<"Compressing image"<<std::endl;
  int size = image.height*image.width;
  int numColours = 4;
  //these are the colours each pixel can be feel free to change or add more
  double centroids[numColours][N] = {
    {125, 149, 57}, //1.21
    {255, 255, 255}, //1
    {226, 46, 1}, //9.61
    {242, 192, 0} //2.5
  };
  double** X = loadImageToMatrix(image);
  double A [numColours];
  double ratioCentroids [numColours];
  double min = std::numeric_limits<double>::max();
  int minArg = 0;
  double modifiedX [size][N];
  /* Takes a ratio of red to green and blue for each pixel
     and works out which each individual pixel matches what ratio
  */
  for(int i = 0; i < 2; i++){
    ratioCentroids[i] = centroids[i][0]/((centroids[i][1]+centroids[i][2])/2);
  }
  for(int i = 0; i < size; i++){
    min = std::numeric_limits<double>::max();
    double ratioX = X[i][0]/((X[i][1]+X[i][2])/2);
    for(int j = 0; j < numColours; j++){
      A[j] = pow(ratioX - ratioCentroids[j], 2);
      if(A[j] < min){
        min = A[j];
        minArg = j;
      }
    }
    for(int j = 0; j < N; j++){
      modifiedX[i][j] = centroids[minArg][j];
    }
  }

  for(int row = 0; row < image.height; row++){
    for ( int column = 0; column < image.width ; column++) {
      set_pixel(image, row, column, modifiedX[image.width*row+column][0], modifiedX[image.width*row+column][1], modifiedX[image.width*row+column][2]);
    }
  }
}

/** Calculates the error of the path taken from the white line*/
double findError(ImagePPM& image){
  std::cout<<"Finding Error"<<std::endl;
  double error = 0;
  double middlePix[] = {(double)image.width/2, (double)image.height}; //reference pixel for error
  double** X = loadImageToMatrix(image);
  for(int row = 0; row < image.height; row++){
    for(int column = 0; column < image.width; column++){
      bool isWhite = true;
      int difference = 0;
      for(int i = 0; i < N; i++){
        if((X[image.width*row+column][i] == 255)&&isWhite){
          isWhite = true;
        }
        else{
          isWhite = false;
        }
      }
      if (isWhite){
        difference = column - middlePix[0];
      }
      error = error + difference;
    }
  }
  return error;
}

int main(){
	if (initClientRobot() !=0){
		std::cout<<" Error initializing robot"<<std::endl;
	}
  std::cout<<"Robot initialized"<<std::endl;
  double vLeft = 10.0;
  double vRight = 10.0;
  takePicture();
  compressImage(cameraView);
  SavePPMFile("i0.ppm",cameraView);
  while(1){
    takePicture();
    compressImage(cameraView);
    double error = findError(cameraView)/10000;
    setMotors(vLeft + error,vRight - error);
    std::cout<<" vLeft="<<vLeft + error<<"  vRight="<<vRight - error<<std::endl;
    usleep(10000);
    std::cout<<"\e[1;1H\e[2J";
  } //while

} // main
