
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
  int numColours = 5;
  //these are the colours each pixel can be feel free to change or add more
  double centroids[numColours][N] = {
    {125, 149, 57}, //1.21
    {255, 255, 255}, //1
    {255, 1, 1},    //255
    {242, 192, 0}, //2.5
    {0, 0, 0} //black, special case, always last in set
  };
  double** X = loadImageToMatrix(image);
  double A [numColours];
  double ratioCentroids [numColours-1];
  double min = std::numeric_limits<double>::max();
  int minArg = 0;
  double modifiedX [size][N];
  /* Takes a ratio of red to green and blue for each pixel
     and works out which each individual pixel matches what ratio
  */
  for(int i = 0; i < numColours-1; i++){
    ratioCentroids[i] = centroids[i][0]/((centroids[i][1]+centroids[i][2])/2.0);
  }
  for(int i = 0; i < size; i++){
    min = std::numeric_limits<double>::max();
    for(int j = 0; j < N; j++){
      if(X[i][j] == 0){
        X[i][j] = 1;
      }
    }
    double ratioX = X[i][0]/((X[i][1]+X[i][2])/2.0);
    if(X[i][0] == X[i][1] == X[i][2] == 1){ //fix to stop robot seeing black as white
      minArg = numColours - 1;
    }
    else{
      for(int j = 0; j < numColours-1; j++){
        A[j] = pow(ratioX - ratioCentroids[j], 2);
        if(A[j] < min){
          min = A[j];
          minArg = j;
        }
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
  SavePPMFile("compressed.ppm",cameraView);
}

/** Calculates the error of the path taken from the white line*/
double findWhiteError(ImagePPM& image){
  std::cout<<"Finding White Error"<<std::endl;
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

double findRedError(ImagePPM& image){
  std::cout<<"Finding Red Error"<<std::endl;
  double error = 0.0;
  double middlePix[] = {(double)image.width/2, (double)image.height}; //reference pixel for error
  double** X = loadImageToMatrix(image);
  for(int row = 25; row < image.height; row++){
    for(int column = 0; column < image.width; column++){
      bool isRed = true;
      double difference = 0.0;
      
      if((X[image.width*row+column][0]==255)&&(X[image.width*row+column][1]==1)&&isRed){
        isRed = true;
      }
      else{
        isRed = false;
      }
      if (isRed){
        difference = (column - middlePix[0]);
      }
      error = error + difference;
    }
  }
  return error;
}
/** 
 * Checks to see if the path is blocked by a wall
 */
bool pathBlocked(ImagePPM& image){
  double middlePix[] = {(double)image.width/2, (double)image.height}; //reference pixel for error
  double** X = loadImageToMatrix(image);
  bool pathBlocked = false;
  for(int row = middlePix[1]-50; row < image.height; row++){
    for(int column = middlePix[0]-10; column < middlePix[0] + 10; column++){
      bool isRed = true;
      double difference = 0.0;
      
      if((X[image.width*row+column][0]==255)&&(X[image.width*row+column][1]==1)&&isRed){
        isRed = true;
      }
      else{
        isRed = false;
      }
      if (isRed){
        pathBlocked = true;
      }
    }
  }
  return pathBlocked;
}

/** 
 * Counts how many cycles the robot has been turning left or right
 */ 
int counter(int count, double totalError){
  if (totalError > 0.5){
    if (count < 0){
      count = 0;
    }
    count++;
  }
  else if(totalError < 0.5){
    if(count > 0){
      count = 0;
    }
    count--;
  }
  return count;
}

/**
 * Returns 1 if robot should turn right, -1 if robot should turn left
 */
int turnControl(ImagePPM& image, double totalError, int count, int turn){
  std::cout<<"Turn: "<<turn<<" Count: "<<count<<std::endl;
  if ((count > 0)&&!pathBlocked(cameraView)&&count>4){
    turn = 1;
  }
  else if ((count < 0)&&!pathBlocked(cameraView)&&count< -4){
    turn = -1;
  }
  return turn;
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
  int turn = 1;
  int count = 0;
  while(1){
    takePicture();
    compressImage(cameraView);
    double whiteError = findWhiteError(cameraView)/10000;
    double redError = findRedError(cameraView)/50000;
    double totalError = whiteError - redError;
    count = counter(count, totalError);
    turn = turnControl(cameraView, totalError, count, turn);
    if(pathBlocked(cameraView)){
      vRight = turn*-5.0;
      vLeft = turn*5.0;
      std::cout<<"Path Blocked"<<std::endl; 
    }
    else{
      vRight = 10.0;
      vLeft = 10.0;
      vLeft = vLeft + totalError*vLeft;
      vRight = vRight - totalError*vRight;      
    }
    setMotors(vLeft,vRight);
    std::cout<<"Error: "<<totalError<<std::endl;
    std::cout<<" vLeft="<<vLeft<<"  vRight="<<vRight<<std::endl;
    usleep(10000);
    std::cout<<"\e[1;1H\e[2J"; //clears the screen
  } //while

} // main

