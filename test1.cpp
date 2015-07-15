#include<iostream>
#include<math.h>
#include<vector>
#include<valarray>
#include<fstream>
#include<stdio.h>
#include<time.h>
#define PI 3.14159265
using namespace std;

ofstream outfile("outmain1.dat");
int plotcode = 0;

class Object
{
public:
	void setShape(float l, float b, float h)
	{
		length = l;
		breadth = b;
		height = h;
	}
	void setPosition(float x_coordinate, float y_coordinate, float z_coordinate)
	{
		position[0] = x_coordinate;
		position[1] = y_coordinate;
		position[2] = z_coordinate;
	}
	void setProperty(float epsilon, float mu)
	{
		e = epsilon;
		u = mu;
		r_coeff = 0.5;
		t_coeff = 0.8;
		r_index = epsilon * mu;
	}
	valarray <float> getEquations(int i)
	{
		valarray<float> eq(4);
		eq[0] = 0; eq[1] = 0; eq[2] = 1; eq[3] = -position[2] + 0.5*height;
		ans.push_back(eq); eq.resize(4);
		eq[0] = 0; eq[1] = 0; eq[2] = 1; eq[3] = -position[2] - 0.5*height;
		ans.push_back(eq); eq.resize(4);
		eq[0] = 0; eq[1] = 1; eq[2] = 0; eq[3] = -position[1] + 0.5*breadth;
		ans.push_back(eq); eq.resize(4);
		eq[0] = 0; eq[1] = 1; eq[2] = 0; eq[3] = -position[1] - 0.5*breadth;
		ans.push_back(eq); eq.resize(4);
		eq[0] = 1; eq[1] = 0; eq[2] = 0; eq[3] = -position[0] + 0.5*length;
		ans.push_back(eq); eq.resize(4);
		eq[0] = 1; eq[1] = 0; eq[2] = 0; eq[3] = -position[0] - 0.5*length;
		ans.push_back(eq); eq.resize(4);
		return ans[i];
	}
	//---------------------------------------------!!!!TO DO!!!!---redefine the above and below class functions to account for rotation of objects
	void getPoints()
	{
		float sym[2][4] = { { 1.0, -1.0, -1.0, 1.0 }, { 1.0, 1.0, -1.0, -1.0 } };
		float sym2[2] = { -1.0, 1.0 };
		float dim[3] = { length, breadth, height };
		for (int ii = 0; ii < 3; ii++)
		{
			int i[3] = { ii % 3, (ii + 1) % 3, (ii + 2) % 3 };
			for (int k = 0; k < 2; k++)
			{
				outfile << "planes";
				for (int j = 0; j < 4; j++)
				{
					outfile << " " << i[0] << " " << position[i[0]] + sym2[k] * dim[i[0]] / 2;
					outfile << " " << i[1] << " " << position[i[1]] + sym[0][j] * dim[i[1]] / 2;
					outfile << " " << i[2] << " " << position[i[2]] + sym[1][j] * dim[i[2]] / 2;
				}
				outfile << endl;
			}
		}
	}
	vector< valarray <float> > ans;
	float length, breadth, height;
	float e, u;
	float r_coeff, t_coeff, r_index;
	valarray<float> position = valarray<float>(4);
};

class Receiver
{
public:
	Receiver(float x_a, float y_b, float z_c, float r)
	{
		point[0] = x_a;
		point[1] = y_b;
		point[2] = z_c;
		radius = r;
	}
	void setPoint(float x_a, float y_b, float z_c)
	{
		point[0] = x_a;
		point[1] = y_b;
		point[2] = z_c;
	}
	void setRadius(float r)
	{
		radius = r;
	}
	valarray<float> point = valarray<float>(4);
	float radius;
};

class Transmitter
{
public:
	Transmitter(float x_a, float y_b, float z_c)
	{
		x = x_a;
		y = y_b;
		z = z_c;
	}
	float x, y, z;
};

Receiver receiver(-3, -3, -2.5, 1.0);

Transmitter transmitter(0.0, 0.0, 5.0);

class Ray
{
public:
	void setPoint(float x_a, float y_b, float z_c)
	{
		point[0] = x_a;
		point[1] = y_b;
		point[2] = z_c;
	}
	void setDirection(float x_p, float y_q, float z_r)
	{
		direction[0] = x_p;
		direction[1] = y_q;
		direction[2] = z_r;
	}
	valarray<float> point = valarray<float>(4);
	valarray<float> direction = valarray<float>(4);
};

int isItInside(valarray<float> point, Object obj)
{
	int check = 0;
	float acc1, acc2;
	for (int i = 0; i<6; i++)
	{
		valarray<float> woo1 = obj.getEquations(i) * obj.position;
		valarray<float> woo2 = obj.getEquations(i) * point;
		float acc1 = obj.getEquations(i)[3] + woo1.sum();
		float acc2 = obj.getEquations(i)[3] + woo2.sum();
		if (acc1*acc2 >= 0)
		{
			check += 0;
		}
		else
		{
			check += 1;
		}
	}
	if (check == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

valarray<float> getPOI(Ray ray, Object obj)
{
	float t = 0; int i_min = -1; int count = 0;
	for (int ii = 0; ii<6; ii++)
	{
		float acc1 = (obj.getEquations(ii) * ray.point).sum();
		float acc2 = (obj.getEquations(ii) * ray.direction).sum();
		float tt;
		tt = -1.0*(obj.getEquations(ii)[3] + acc1) / acc2;
		if (tt > 0 && isItInside(ray.point + ray.direction * tt, obj) == 1){
			if (count == 0)
			{
				i_min = ii;
				t = tt;
			}
			else if (tt < t)
			{
				i_min = ii;
				t = tt;
			}
			count++;
		}
	}
	valarray<float> ip(5); ip[3] = t; ip[4] = (float)i_min;
	valarray<float> safe_ip(3);
	safe_ip = ray.point + ray.direction*t;
	for (int i = 0; i < 3; i++)
	{
		ip[i] = safe_ip[i];
	}
	return ip;
}

float isSameSide(valarray<float> p1, valarray<float> p2, valarray<float> plane)
{
	float check = (plane[0] * p1[0] + plane[1] * p1[1] + plane[2] * p1[2] + plane[3])*(plane[0] * p2[0] + plane[1] * p2[1] + plane[2] * p2[2] + plane[3]);
	if (check > 0)
	{
		return 1.0;
	}
	else
	{
		return -1.0;
	}
}

valarray<float> doesItPass(Ray ray, Receiver rec)
{
	valarray<float> ans(2);
	ans[0] = ((rec.point - ray.point) * (ray.direction)).sum() / (ray.direction * ray.direction).sum();
	valarray<float> p = ray.point - rec.point + ans[0] * ray.direction;
	if (sqrt((p * p).sum()) <= rec.radius && ans[0] > 0.0) ans[1] = 1;
	else ans[1] = 0;
	return ans;
}

valarray<float> nextObject(int presentIndex, Ray ray, vector<Object> obstacles)
{
	int count = 0, index = -1, iii = -1;
	float t = 0.0;
	for (size_t i = 0; i<obstacles.size(); i++)
	{
		valarray<float> poi = getPOI(ray, obstacles[i]);
		valarray<float> poi_4(4); for (int poi_i = 0; poi_i < 3; poi_i++) poi_4[poi_i] = poi[poi_i];
		//cout << poi_4[0] << "\t" << poi_4[1] << "\t" << poi_4[2] << endl;
		float tt = poi[3];
		float ii = poi[4];
		if (tt > 0.0)
		{
			if (count == 0)
			{
				index = i;
				t = tt;
				iii = (int)ii;
			}
			else if (presentIndex == i);
			else if (tt < t)
			{
				index = i;
				t = tt;
				iii = (int)ii;
			}
			count++;
		}
	}
	valarray<float> ans(3);
	ans[0] = t;
	ans[1] = (float)iii;
	ans[2] = (float)index;
	return ans;
}

int raytrace(Ray ray, float fieldStrength, float pathLength, vector<Object> obstacles, int presentIndex)
{
	int index, iii;
	float t = 0.0;

	//find the next obstacle in the path of the ray:
	valarray<float> indices = nextObject(-1, ray, obstacles);
	t = indices[0];
	iii = (int)indices[1];
	index = (int)indices[2];

	if (t == 0)
	{
		cout << "DONE1!!" << endl;
		return 1;//<----------------------------------!!!!TO DO!!!!---return point where ray dies------------------------------------------------
	}

	//check if it enters receiver region:
	valarray<float> receiverCheck = doesItPass(ray, receiver);
	int didItReach = 0;
	if (receiverCheck[1] == 1 && receiverCheck[0] < t)
	{
		t = receiverCheck[0];
		didItReach = 1;
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---debug the receiver region::ray is not stopping even after entering the region

	//find the point of intersection and set it for next rays:
	valarray<float> p;
	p = ray.point + t * ray.direction;
	pathLength += t * sqrt((ray.direction * ray.direction).sum());
	Ray reflectedRay, transmittedRay, bufferRay;
	reflectedRay.setPoint(p[0], p[1], p[2]);
	transmittedRay.setPoint(p[0], p[1], p[2]);
	float small_t = 0.01;
	valarray<float> pp = p + small_t * ray.direction;
	int nextIndex = 0, count_lo = 0;
	for (size_t lo = 0; lo < obstacles.size(); lo++)
	{
		if (isItInside(pp, obstacles[lo]) == 1 && lo != 0)
		{
			nextIndex = lo;
		}
	}

	//did it reach?
	if (didItReach == 1)
	{
		float timeOfReach = pathLength / (3e8);
		outfile << "Time " <<  timeOfReach << " " << fieldStrength << endl;
		outfile << ray.point[0] << " " << ray.point[1] << " " << ray.point[2] << " " << plotcode << endl;
		outfile << p[0] << " " << p[1] << " " << p[2] << " " << plotcode << endl;
		plotcode++;
		return 2;
	}

	//if field falls below threshold (set as 0.1) then stop and display point
	if (fieldStrength<0.1)
	{
		cout << "DONE2!!" << endl;
		return 1;
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---find exact point where the ray dies----------------------------------------

	//set the direction ratios of the reflected ray here:
	float acc1 = (ray.direction * ray.direction).sum();
	float acc2 = -1.0*((ray.direction * obstacles[index].getEquations(iii)).sum());
	float acc3 = (obstacles[index].getEquations(iii) * obstacles[index].getEquations(iii)).sum() - pow(obstacles[index].getEquations(iii)[3], 2);
	t = acc2 / acc1;
	valarray<float> normal_check1 = p + small_t * ray.direction;
	valarray<float> normal_check2 = p + small_t * obstacles[index].getEquations(iii);
	float f_normal = -1.0 * isSameSide(normal_check1, normal_check2, obstacles[index].getEquations(iii));
	float i_dot_n = 2.0 * acc2;// * f_normal;
	valarray<float> ans = i_dot_n * obstacles[index].getEquations(iii) + ray.direction;
	reflectedRay.setDirection(ans[0], ans[1], ans[2]);

	//set the direction ratios of the transmitted ray here:
	float cosTheta1 = -acc2 / sqrt(acc1*acc3); float sinTheta1 = sin(acos(cosTheta1));
	float n = obstacles[presentIndex].r_index / obstacles[nextIndex].r_index;
	float sinTheta2 = n*sinTheta1; float cosTheta2 = cos(asin(sinTheta2));
	float t_c = -1.0*sqrt(1 - pow(sinTheta2, 2)) / sqrt(acc3);
	valarray<float> t_par = n*((ray.direction / sqrt(acc1)) + (cosTheta1 * f_normal * obstacles[index].getEquations(iii) / sqrt(acc3)));
	valarray<float> t_per = t_c * f_normal * obstacles[index].getEquations(iii);
	valarray<float> t_total = t_par + t_per;
	transmittedRay.setDirection(t_total[0], t_total[1], t_total[2]);

	//trace the reflected and refracted rays next:
	int ref_return = 1;
	// if (index == 0 || presentIndex == 0)
	// {
	// 	cout << "\nSTARTING REFLECTION\n@ " << reflectedRay.point[0] << " " << reflectedRay.point[1] << " " << reflectedRay.point[2] << endl;
	// 	ref_return = raytrace(reflectedRay, obstacles[presentIndex].r_coeff * fieldStrength, pathLength, obstacles, presentIndex);
	// }
	// cout << "\nSTARTING REFRACTION\n@ " << transmittedRay.point[0] << " " << transmittedRay.point[1] << " " << transmittedRay.point[2] << endl;
	int trans_return = raytrace(transmittedRay, obstacles[index].t_coeff * fieldStrength, pathLength, obstacles, nextIndex);
	// if( ref_return * trans_return >= 2 )
	// {
		outfile << ray.point[0] << " " << ray.point[1] << " " << ray.point[2] << " " << plotcode << endl;
		outfile << p[0] << " " << p[1] << " " << p[2] << " " << plotcode <<endl;
		plotcode++; 
	// }
	return ref_return * trans_return;
}

int main()
{
	time_t start = time(0);
	outfile << "# Started @ " << start << endl;
	outfile << "Receiver " << receiver.point[0] << " " << receiver.point[1] << " " << receiver.point[2] << " " << receiver.radius << endl;
	outfile << "Transmitter " << transmitter.x << " " << transmitter.y << " " << transmitter.z << endl;

	vector<Object> obstacles;
	Object Room;
	Room.setShape(10, 10, 10);
	Room.setPosition(0, 0, 0);
	Room.setProperty(1, 1);
	Room.getPoints();//check once
	obstacles.push_back(Room);

	Object Box;
	Box.setShape(2, 10, 10);
	Box.setPosition(3, 0, 0);
	Box.setProperty(2, 1);
	Box.getPoints();
	obstacles.push_back(Box);

	//	Object Box2;
	//	Box2.setShape(0.9, 0.9, 0.9);
	//	Box2.setPosition(0.5, 0.5, 0.5);
	//	Box2.setProperty(1, 1);
	//	Box2.getPoints();
	//	obstacles.push_back(Box2);

	Object Box3;
	Box3.setShape(3, 4, 4);
	Box3.setPosition(-2.5, 0, -3);
	Box3.setProperty(2, 2);
	Box3.getPoints();
	obstacles.push_back(Box3);

	// Ray ray;
	// ray.setPoint(transmitter.x, transmitter.y, transmitter.z);
	// ray.setDirection(1, 0, -3);
	// raytrace(ray, 1, 0, obstacles, 0);

	// outfile << endl;

	// Ray ray2;
	// ray2.setPoint(transmitter.x, transmitter.y, transmitter.z);
	// ray2.setDirection(3, 0, -2);
	// raytrace(ray2, 1, 0, obstacles, 0);

	// outfile << endl;

	// Ray ray3;
	// ray3.setPoint(transmitter.x, transmitter.y, transmitter.z);
	// ray3.setDirection(1, -1, -1);
	// raytrace(ray3, 1, 0, obstacles, 0);

	//somehow start many rays from transmitter
	// for(int radius = 1; radius <= 5; radius+=1 )
	// {
	// 	for(float angle = 0; angle <= 2 * PI; angle += 0.25)
	// 	{
	// 		Ray rayX;
	// 		rayX.setPoint(transmitter.x, transmitter.y, transmitter.z);
	// 		rayX.setDirection(radius * cos(angle), radius * sin(angle), -5.0);
	// 		raytrace(rayX, 1, 0, obstacles, 0);
	// 	}
	// }

	float fR = 0.6, fA = 0.1, fB = 0.1;
	for(float fi = -fB * (int)(fR/fB); fi <= fR; fi = fi + fB)
	{
		float fr = sqrt(pow(fR,2) - pow(fi,2));
		for(float fa = 0; fa <= 2 * PI; fa = fa + fA/fr)
		{
			Ray rayX;
			rayX.setPoint(transmitter.x, transmitter.y, transmitter.z);
			rayX.setDirection(fr * cos(fa), fr * sin(fa), fi);
			raytrace(rayX, 1, 0, obstacles, 0);
		}
	}
	outfile << "# Ending @ " << time(0) << endl;
	cout << "# Time elasped: " << difftime( time(0), start) << endl;
	outfile.close();
	//getchar();
	return 0;
}

/*ALGO*/
//it sould contain info about the ray as: (a,b,c)+t(p,q,r)
//say the receiver is at (rx,ry,rz) with radius of say rr units
//first find the intersection point-'ip' and its parameter 't'
//then find the point nearest to the receiver and check if it comes before 'ip'
//-->YES?:find the signal strength after attenuation at that nearest point and add it to main field
//find the length travelled from (a,b,c) till that point, add it to its total path length variable, use it later
//then terminate the ray
//-->NO ?:calculate the angle of incidence of the ray onto the next plane it intersects
//calculate the angle of refraction of the ray transmitted through the plane
//calculate amount of attenuation that the ray undergoes till 'ip'; change the field strength variable accordingly
//raytrace two new rays from the 'ip'--one transmitted:(a,b,c):ip;(p,q,r):find from the angle and plane of incidence
//--other reflected:(a,b,c):ip;(p,q,r):find from angle and plane of incidence
//also pass on the total path length variable onto the next generation rays
//cl /EHsc basic.cpp