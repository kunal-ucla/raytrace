package main

import "fmt"

type Object struct{
	  length,breadth,heightfloat32
	  position = []float32
	  r_coeff,t_coeff,r_indexfloat32
}



func getEquations(i int, obj Object) []float32{

	var ans []float32 
	switch i{
	case 0:
		  ans = []float32 {0,0,1,-obj.position[2] + 0.5*obj.height}
	case 1:
		  ans = []float32 {0,0,1,-obj.position[2] - 0.5*obj.height}
	case 2:
		  ans = []float32 {0,1,0,-obj.position[1] + 0.5*obj.breadth}
	case 3:
		  ans = []float32 {0,1,0,-obj.position[1] - 0.5*obj.breadth}
	case 4:
		var ans = []float32 {1,0,0,-obj.position[0] + 0.5*obj.length}
	case 5:
		  ans = []float32 {1,0,0,-obj.position[0] - 0.5*obj.length}
	}
	return ans
}

func dot(a []float32, bfloat32) []float32{
	var c = []float32 {0,0,0}
	for i:=0;i<3;i++{
		c[i]=a[i]*b
	}
	return c
}

func sum(a []float32)float32{
	var cfloat32
	for i:=0;i<3;i++{
		c=c+a[i]
	}
	return c
}

func dot2(a,b []float32) []float32{
	var c = []float32 {0,0,0}
	for i:=0;i<3;i++{
		c[i]=a[i]*b[i]
	}
	return c
}

func sum2(a,b []float32) []float32{
	var c = []float32 {0,0,0}
	for i:=0;i<3;i++{
		c[i]=a[i]+b[i]
	}
	return c
}

func getPoints(obj Object) void{
	
	var sym = [][]float32 { { 1.0, -1.0, -1.0, 1.0 }, { 1.0, 1.0, -1.0, -1.0 } }
	var sym2 = []float32 = { -1.0, 1.0 }
	var dim = []float32 = { obj.length, obj.breadth, obj.height }
	for ii := 0; ii < 3; ii++ {
		var i = [] int = { ii % 3, (ii + 1) % 3, (ii + 2) % 3 }
		for k := 0; k < 2; k++ {
			outfile << "planes";//gogogogogogogo
			for j := 0; j < 4; j++ {
				outfile << " " << i[0] << " " << obj.position[i[0]] + sym2[k] * dim[i[0]] / 2;//gogogogogogogo
				outfile << " " << i[1] << " " << obj.position[i[1]] + sym[0][j] * dim[i[1]] / 2;//gogogogogogogo
				outfile << " " << i[2] << " " << obj.position[i[2]] + sym[1][j] * dim[i[2]] / 2;//gogogogogogogo
			}
			outfile << endl;//gogogogogogogo
		}
	}
}

type Receiver struct{
	var point = []float32
	var radiusfloat32
}

type Transmitter struct{
	var point = []float32
}

type Ray struct{
	var point = []float32
	var direction = []float32
}

func isItInside(point []float32, obj Object) int{
	check := 0
	acc1, acc2float32
	for i = 0; i < 6; i++{
		woo1 := dot2(getEquations(i,obj), obj.position)
		woo2 := dot2(getEquations(i,obj), point)
		acc1 := getEquations(i,obj)[3] + sum(woo1)
		acc2 := getEquations(i,obj)[3] + sum(woo2)
		if acc1*acc2 >= 0{
			//do nothing
		} else{
			check++
		}
	}
	if check == 0{
		return 1
	} else{
		return 0
	}
}

func getPOI(ray Ray, obj Object) []float32{
	t := 0.0
	i_min := -1
	count := 0
	for ii:=0;ii<6;ii++{
		acc1 := sum(dot2(getEquations(ii,obj),ray.point))
		acc2 := sum(dot(getEquations(ii,obj),ray.direction))
		tt := -1.0*(getEquations(ii,obj)[3] + acc1) / acc2;
		if tt > 0 && isItInside(sum2(ray.point,dot(ray.direction,tt),obj)) == 1 {
			if count == 0 {
				i_min = ii;
				t = tt;
			} else if tt < t {
				i_min = ii;
				t = tt;
			}
			count++;
		}
	}
	var ip = []float32 {0,0,0,0,0}
	ip[3] = t
	ip[4] =float32(i_min)
	var safe_ip = []float32 {0,0,0}
	safe_ip = sum2(ray.point,dot(ray.direction,t))
	for i:=0;i<3;i++ {
		ip[i] = safe_ip[i]
	}
	return ip
}

func isSameSide(p1 []float32, p2 []float32, plane []float32)float32{
	check := (plane[0] * p1[0] + plane[1] * p1[1] + plane[2] * p1[2] + plane[3])*(plane[0] * p2[0] + plane[1] * p2[1] + plane[2] * p2[2] + plane[3])
	if check > 0 {
		return 1.0
	} else {
		return -1.0
	}
}

func doesItPass(ray Ray, rec Receiver) []float32{
	var ans = []float32 {0,0}
	ans[0] = sum(dot2(sum2(rec.point,dot(ray.point,-1.0)),ray.direction)) / sum(dot2(ray.direction,ray.direction))
	p := sum2(sum2(ray.point,dot(rec.point,-1.0)),dot(ray.direction,ans[0]))
	if Sqrt(sum(dot2(p,p))) <= rec.radius && ans[0] > 0.0 {
		ans[1] = 1
	} else {
		ans[1] = 0
	}
	return ans
}

func nextObject(presentIndex int, ray Ray, obstacles []Object) []float32{
	count := 0
	index := -1
	iii := -1
	t := 0.0
	for i := 0;i<len(obstacles);i++	{
		poi := getPOI(ray, obstacles[i])
		var poi_4 = []float32 {0,0,0,0}
		for poi_i := 0;poi_i<3;poi_i++ {
			poi_4[poi_i]=poi[poi_i]
		}
		//cout << poi_4[0] << "\t" << poi_4[1] << "\t" << poi_4[2] << endl;
		tt := poi[3]
		ii := poi[4]
		if tt>0.0 {
			if count==0 {
				index = i
				t = tt
				iii = int(ii)
			} else if presentIndex==i {
				//do nothing
			} else if tt<t {
				index = i
				t = tt
				iii = int(ii)
			}
			count++
		}
	}
	var ans = []float32 {0,0,0}
	ans[0] = t
	ans[1] =float32(iii)
	ans[2] =float32(index)
	return ans
}

func raytrace(ray Ray, fieldStrengthfloat32, pathLengthfloat32, obstacles []Object, presentIndex int) int{
	var index, iii int
	t := 0.0

	//find the next obstacle in the path of the ray:
	indices := nextObject(-1, ray, obstacles)
	t = indices[0]
	iii = int(indices[1])
	index = int(indices[2])

	if t==0 {
		return 1//<----------------------------------!!!!TO DO!!!!---return point where ray dies------------------------------------------------
	}

	//check if it enters receiver region:
	receiverCheck := doesItPass(ray, receiver)
	didItReach := 0
	if receiverCheck[1]==1 && receiverCheck[0]<t {
		t = receiverCheck[0]
		didItReach = 1
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---debug the receiver region::ray is not stopping even after entering the region

	//find the point of intersection and set it for next rays:
	p := sum2(ray.point, dot(ray.direction,t))
	pathLength = pathLength + t * Sqrt(sum2(ray.direction, ray.direction))
	//var bufferRay Ray
	reflectedRay:= Ray(point:p)
	transmittedRay:= Ray(point:p)
	small_t := 0.01
	pp := sum2(p, dot(ray.direction,small_t))
	nextIndex := 0
	count_lo := 0
	for lo:=0;lo<len(obstacles);lo++ {
		if isItInside(pp, obstacles[lo])==1 && lo!=0 {
			nextIndex = lo
		}
	}
	ppp := sum2(p, dot(ray.direction,-small_t))
	prevIndex := 0
	count_lo = 0
	for lo:=0;lo<len(obstacles);lo++ {
		if isItInside(pp, obstacles[lo]) == 1 && lo != 0 {
			prevIndex = lo
		}
	}

	//did it reach?
	if didItReach == 1 {
		timeOfReach := pathLength / (3e8)
		outfile << "Time " <<  timeOfReach << " " << fieldStrength << endl;//gogogogogogogo
		outfile << ray.point[0] << " " << ray.point[1] << " " << ray.point[2] << " " << plotcode << endl;//gogogogogogogo
		outfile << p[0] << " " << p[1] << " " << p[2] << " " << plotcode << endl;//gogogogogogogo
		plotcode++
		return 2
	}

	//field attenuation with distance:
	fieldStrength *= exp(-1 * pathLength * obstacles[prevIndex].gamma);

	//if field falls below threshold (set as 0.1) then stop
	if fieldStrength<0.1 {
		return 1
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---find exact point where the ray dies----------------------------------------

	//set the direction ratios of the reflected ray here:
	acc1 := sum2(ray.direction, ray.direction)
	acc2 := -1.0*(sum(dot2(ray.direction, getEquations(iii,obstacles[index]))))
	acc3 := sum(dot2(obstacles[index].getEquations(iii), obstacles[index].getEquations(iii))) - Pow(getEquations(iii,obstacles[index])[3], 2)
	t = acc2 / acc1
	normal_check1 := sum2(p, dot2(small_t, ray.direction))
	normal_check2 := sum2(p, dot2(small_t, getEquations(iii,obstacles[index])))
	f_normal := -1.0 * isSameSide(normal_check1, normal_check2, getEquations(iii,obstacles[index]));
	i_dot_n := 2.0 * acc2;// * f_normal;
	ans := dot2(i_dot_n, sum2(getEquations(iii,obstacles[index]), ray.direction))
	reflectedRay.direction = ans

	//set the direction ratios of the transmitted ray here:
	cosTheta1 := -acc2 / Sqrt(acc1*acc3)
	sinTheta1 := Sin(Acos(cosTheta1))
	n := obstacles[presentIndex].r_index / obstacles[nextIndex].r_index
	sinTheta2 := n*sinTheta1
	cosTheta2 := Cos(Asin(sinTheta2))
	t_c := -1.0*Sqrt(1 - Pow(sinTheta2, 2)) / Sqrt(acc3)
	lollipop := f_normal * cosTheta1 / Sqrt(acc3)
	t_par := dot(sum2(dot(ray.direction ,1/sqrt(acc1)), dot(getEquations(iii,obstacles[index]), lollipop)),2)
	t_per := dot(getEquations(iii,obstacles[index]), f_normal * t_c)
	t_total := sum2(t_par, t_per)
	transmittedRay.direction = t_total

	//trace the reflected and refracted rays next:
	ref_return := 1
	if index == 0 || presentIndex == 0 {
		ref_return = raytrace(reflectedRay, obstacles[presentIndex].r_coeff * fieldStrength, pathLength, obstacles, presentIndex)
	}
	trans_return := raytrace(transmittedRay, obstacles[index].t_coeff * fieldStrength, pathLength, obstacles, nextIndex)
	if ref_return * trans_return >= 2 {
		outfile << ray.point[0] << " " << ray.point[1] << " " << ray.point[2] << " " << plotcode << endl;//gogogogogogogo
		outfile << p[0] << " " << p[1] << " " << p[2] << " " << plotcode <<endl;//gogogogogogogo
		plotcode++
	}
	return ref_return * trans_return
}

func main() {
	outfile << "Receiver " << receiver.point[0] << " " << receiver.point[1] << " " << receiver.point[2] << " " << receiver.radius << endl;//gogogogogogogo
	outfile << "Transmitter " << transmitter.x << " " << transmitter.y << " " << transmitter.z << endl;//gogogogogogogo

	//var obstacles []Object
	Room := Object{length:13,breadth:8.6,height:3,r_coeff:0.8,t_coeff:0,r_index:1,position:[]float32{0,0,0}}
	getPoints(Room)

	Box := Object{length:0.03,breadth:2.5,height:3,r_coeff:0.8,t_coeff:0,r_index:2,position:[]float32{-3.6586,-1.72,0}}
	getPoints(Box)

	Box1 := Object{length:0.91,breadth:0.645,height:3,r_coeff:0.8,t_coeff:0,r_index:2,position:[]float32{6.045,3.9775,0}}
	getPoints(Box1)

	Box2 := Object{length:0.91,breadth:0.645,height:3,r_coeff:0.8,t_coeff:0,r_index:2,position:[]float32{-6.045,3.9775,0}}
	getPoints(Box2)

	Box3 := Object{length:0.91,breadth:0.645,height:3,r_coeff:0.8,t_coeff:0,r_index:2,position:[]float32{6.045,-3.9775,0}}
	getPoints(Box3)

	Box4 := Object{length:0.91,breadth:0.645,height:3,r_coeff:0.8,t_coeff:0,r_index:2,position:[]float32{-6.045,-3.9775,0}}
	getPoints(Box4)

	var obstacles []Object {Room, Box, Box1, Box2, Box3, Box4}

	//somehow start many rays from transmitter
	fR := 0.6
	fA := 0.05
	fB := 0.05
	for fi:=-fB*int(fR/fB);fi<=fR;fi=fi+fB {
		fr:=Sqrt(Pow(fR,2) - Pow(fi,2));
		for fa:=0.0;fa<=2*PI;fa=fa+fA/fr {
			rayX:=Ray{point:transmitter.point,direction:[]float32{fr*Cos(fa),fr*Sin(fa),fi}}
			raytrace(rayX, 1, 0, obstacles, 0)
		}
	}

	outfile << "# Ending @ " << time(0) << endl;//gogogogogogogo
	outfile.close();//gogogogogogogo
}