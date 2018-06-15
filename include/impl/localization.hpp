Localization::Localization(){
//	sw.start();
	x = y = pitch = yaw = v = w = 0.0;
    z = 0.0;
	time = 0.0;
}

Localization::~Localization(){
}

void Localization::showState(){
	cout << "x: " << x << ", y: " << y << ", yaw: " << yaw << endl;
}

void Localization::start(){
	sw.start();
}

void Localization::gettime(){
	time = sw.getTime();
	sw.reset();
}

void Localization::altering(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	yaw += dt * w;
	// if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
}
void Localization::altering2(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	// yaw += dt * w; 
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	// x += dt * v * cos(yaw);
	// y += dt * v * sin(yaw);
	x += dt * v * cos(pitch) * cos(yaw);
	y += dt * v * cos(pitch) * sin(yaw);
}

void Localization::altering3(){ //8/16変更 lcl[0],[1],[4]で適用
    
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	//yaw += dt * w;  
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	// x += dt * v * cos(yaw);
	// y += dt * v * sin(yaw);
	//x +=  v * cos(pitch) * cos(yaw);
	//y +=  v * cos(pitch) * sin(yaw);

	// bool flag = true;

	w = d_yaw/dt;

	// if(fabs(w)>0.3){
    //
	// 	flag = false;
    //
	// }

	// if(flag){
		//正解
		x += dt * v * cos(pitch) * cos(yaw);
		y += dt * v * cos(pitch) * sin(yaw);
		z += dt * v * sin(pitch);
	// }

}



void Localization::altering4(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	yaw += dt * w; 
	//if(yaw<-M_PI)yaw += 2*M_PI;
	//if(yaw>M_PI)yaw -= 2*M_PI;
	// x += dt * v * cos(yaw);
	// y += dt * v * sin(yaw);
	x += dt * v * cos(yaw);
	y += dt * v * sin(yaw);
}


void Localization::alter_dyaw(){
	// double dt = 3.0 * sw.getTime();
	double dt = sw.getTime();
	sw.reset();
	w = d_yaw / dt ; 

}
