#include "robot.hpp"
int main(){
	if (initClientRobot() !=0){
		std::cout<<" Error initializing robot"<<std::endl;
	}
    double vLeft;
    double vRight;
    double width = 150;
    int *white = new int[150];
    
    while (1){
		vLeft = 20.0;
		vRight = 20.0;
		takePicture();
		for(int i = 0; i < width; i++){
			int pixel = get_pixel(cameraView, 50, i, 3);
			int isWhite;
			
			if(pixel > 250){
				white[i] = 1;
				isWhite = 1;	
				}
				else{white[i] = 0;
					isWhite = 0;
					}
			}
		for(int i = 0; i < width; i++){
			std::cout << white[i] << std::endl;
			} 
		
		double avrg = 0;
		int j = 0;
		for(int i =0; i < width; i++){
			if(white[i] == 1){
				avrg += i+1;
				j++;
				}
			}
		avrg = avrg/j;
		std::cout << avrg << std::endl;
		
		double turn = 1.2;
		int mid = width/2;
		double error = mid - avrg;
		double Left = vLeft;
		double Right = vRight;
		if(error <= 0){
			vLeft = Left - error*turn;
			}  		
		else{vRight = Right + error*turn;}
		std::cout << std::endl;
		setMotors(vLeft, vRight);
		std::cout<<"Left="<<vLeft<<"Right="<<vRight<<std::endl;
			
       usleep(10000);
  } //while

} // main

 
