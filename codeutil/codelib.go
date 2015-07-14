package codeutil

import (
	"math"
	"time"
)

type ProcessData struct {
	TimeTaken time.Duration
	NumCores  int
}

type JsonData struct {
	Receiver    []float64
	Transmitter []float64
	Planes      [][][]float64
	Time        [][]float64
	Points      [][]float64
	Process     ProcessData
}

var Data JsonData

type Point3D []float64

type Plane3D []Point3D

type Object struct {
	Length, Breadth, Height   float64
	Position                  Point3D
	R_coeff, T_coeff, R_index float64
}

type Receiver struct {
	Point  Point3D
	Radius float64
}

type Transmitter struct {
	Point Point3D
}

type Ray struct {
	Point     Point3D
	Direction []float64
	Id        int
}

func Dot(a []float64, b float64) []float64 {
	/*
		What it does:
		1. Returns the product of a scalar and an array
		2. The array should be of type []float64 only and the scalar should be of type float64 only
	*/
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b
	}
	return c
}

func Sum(a []float64) float64 {
	/*
		What it does:
		1. Returns the sum of the elements of an array
		2. The array should be of type []float64 only
	*/
	var c float64
	for i := 0; i < 3; i++ {
		c = c + a[i]
	}
	return c
}

func Dot2(a, b []float64) []float64 {
	/*
		What it does:
		1. Returns the dot product of two vectors(arrays)
		2. The array should be of type []float64 only
	*/
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b[i]
	}
	return c
}

func Sum2(a, b []float64) []float64 {
	/*
		What it does:
		1. Returns the sum of two vectors(arrays)
		2. The array should be of type []float64 only
	*/
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] + b[i]
	}
	return c
}

func GetPOI(ray Ray, obj Object) []float64 {
	/*
		What it returns:
		1. Returns an array of size 4
		2. First three elements represent the coordinates of the point of intersection of the ray with the object both passed as parameters to this function
		3. The third element represents a parameter 't' which is proportional to the distance between the origin of ray and the point of intersection
		4. This parameter 't' will be helpful in determining which object does the ray hits first:-the one which has the least 't' parameter
	*/
	/*
		Algo:
		1. It iterates through each of the six planes of the given object
		2. Knowing the equation of each of the planes, point of intersection of the ray with each plane can be known
		3. Thus, the 't' parameter(that is proportional to distance b/w POI and ray origin) of each plane can be calculated(and is stored in 'temp_t_of_plane_id_i' variable)
		4. Next we have to know which of the six planes has the least and positive(+ve because the POI has to be along the direction of the ray and not in the backward direction) 't' parameter
		5. The POI of this plane along with its 't' parameter and the id of the plane is thus returned.
	*/
	/*
		What is 't' parameter afterall?:
		1. Consider a ray originating from a 3D point (x,y,z) and travelling along a straight line whose direction is given by its direction ratios say (p,q,r)
		2. Consider a 3D plane in the space. We have to find the point ON this plane through which the above ray passes i.e. their intersection point
		3. We use the fact that any point lying on the given ray can be represented as (x,y,z) + t*(p,q,r) = (x+t*p, y+t*q, z+t*r) where t is any floating point number
		4. Let the intersection point thus be (x+t*p, y+t*q, z+t*r)
		5. Note that greater the magnitude of t, farther it is from the point of origin (x,y,z)
		5. Since this lies on the plane with its plane equation say ax+by+cz+d=0; we can substitute the above point in this and thus find the value of t
		6. We have thus found the intersection point along with a parameter 't' ! We will use this 't' to find the nearest intersection point.
	*/
	t := 0.0
	intersection_plane_index := -1 //This variable will store the index of the plane that the ray intersects (at which of the six planes does the intersection point lie?)
	count := 0
	for i := 0; i < 6; i++ {
		acc1 := Sum(Dot2(obj.GetEquations(i), ray.Point))
		acc2 := Sum(Dot2(obj.GetEquations(i), ray.Direction))
		temp_t_of_plane_id_i := -1.0 * (obj.GetEquations(i)[3] + acc1) / acc2
		if temp_t_of_plane_id_i > 0 && obj.DoesItContain(Sum2(ray.Point, Dot(ray.Direction, temp_t_of_plane_id_i))) == 1 {
			if count == 0 || temp_t_of_plane_id_i < t {
				intersection_plane_index = i
				t = temp_t_of_plane_id_i
			}
			count++
		}
	}
	intersection_point_coordinates := Sum2(ray.Point, Dot(ray.Direction, t))
	var result_to_be_returned = []float64{0, 0, 0, t, float64(intersection_plane_index)}
	for i := 0; i < 3; i++ {
		result_to_be_returned[i] = intersection_point_coordinates[i]
	}
	return result_to_be_returned
}

func IsSameSide(p1 []float64, p2 []float64, plane []float64) float64 {
	/*
		What it returns:
		1. It returns (+1.0) if the given two points p1 and p2 lie on the same side of the given plane
		2. And returns (-1.0) otherwise (even when one of them lies on the plane itself: but that will not happen in our code so it doesn't matter)
	*/
	check := (plane[0]*p1[0] + plane[1]*p1[1] + plane[2]*p1[2] + plane[3]) * (plane[0]*p2[0] + plane[1]*p2[1] + plane[2]*p2[2] + plane[3])
	if check > 0 {
		return 1.0
	} else {
		return -1.0
	}
}

func DoesItPass(ray Ray, rec Receiver) []float64 {
	/*
		What it returns:
		1. It returns an array of size two.
		2. The first element of the array (is similar to the parameter 't') determines the point on the ray nearest to the center of the receiver
		3. The seond element of the array (1 or 0) tells us whether or not the ray falls in the receiver region i.e. whether the above point is@distance<receiverRadius from receiverCenter
	*/
	var result_to_be_returned = []float64{0, 0}
	result_to_be_returned[0] = Sum(Dot2(Sum2(rec.Point, Dot(ray.Point, -1.0)), ray.Direction)) / Sum(Dot2(ray.Direction, ray.Direction))
	p := Sum2(Sum2(ray.Point, Dot(rec.Point, -1.0)), Dot(ray.Direction, result_to_be_returned[0]))
	//The magnitude of the above vector(array) 'p' will determine the shortest distance b/w the ray and the center of the receiver
	if math.Sqrt(Sum(Dot2(p, p))) <= rec.Radius && result_to_be_returned[0] > 0.0 {
		result_to_be_returned[1] = 1
	} else {
		result_to_be_returned[1] = 0
	}
	return result_to_be_returned
}

func NextObject(presentIndex int, ray Ray, obstacles []Object) (float64, int, int) {
	/*
		Algo:
		1. First find the intersection point of the ray with every object in the room (including the room itself as an object) using GetPOI
		2. Then return the least of the 't' parameters collected by the previous step
		3. That will give us the next point from which we have to generate further rays(both transmitted and reflected)
		4. Also return the index of the object and the index of its plane at which the ray is going to fall
		5. Note: In all the cases, the 't' parameter always has to be positive i.e. >0. A negative 't' means a point in the backward direction of the ray.
	*/
	t := 0.0
	next_object_index := 0
	next_plane_index := 0
	count := 0
	for i := 0; i < len(obstacles); i++ {
		poi := GetPOI(ray, obstacles[i])
		t_parameter_of_object_i := poi[3]
		if t_parameter_of_object_i > 0.0 {
			if count == 0 || t_parameter_of_object_i < t {
				t = t_parameter_of_object_i
				next_object_index = i
				next_plane_index = int(poi[4])
			}
			count++
		}
	}
	return t, next_object_index, next_plane_index
}

func (obj Object) GetEquations(i int) []float64 {
	/*
		What it returns:
		0. Note that here 'i' is the index of the plane of the object whose equations are to be determined
		1. It returns the equation of the 'i'th plane of the object 'obj'
		2. The format is []float64{a,b,c,d} if the equation of plane is: ax+by+cz+d=0
		3. Here we have assumed that the object is always cuboidal and its surfaces are parallel to the x, y or z axes.
		4. The above point explains the fact that the first three numbers i.e. {a,b,c} are always from among {0,0,1}
	*/
	var ans []float64
	switch i {
	case 0:
		ans = []float64{0, 0, 1, -obj.Position[2] + 0.5*obj.Height}
	case 1:
		ans = []float64{0, 0, 1, -obj.Position[2] - 0.5*obj.Height}
	case 2:
		ans = []float64{0, 1, 0, -obj.Position[1] + 0.5*obj.Breadth}
	case 3:
		ans = []float64{0, 1, 0, -obj.Position[1] - 0.5*obj.Breadth}
	case 4:
		ans = []float64{1, 0, 0, -obj.Position[0] + 0.5*obj.Length}
	case 5:
		ans = []float64{1, 0, 0, -obj.Position[0] - 0.5*obj.Length}
	}
	return ans
}

func (obj Object) DoesItContain(point Point3D) int {
	/*
		What does it do:
		1. It checks if an object contains a given point inside it or not
		2. And returns 1 if it does, 0 if doesn't
	*/
	/*
		Algo:
		1. It verifies that the given point and the center of the object lie on the SAME side of each of the 6 planes of the object.
		2. If it lies on opposite side of the center w.r.t. atleast one of the 6 planes, then it means that the point is NOT inside the object.
		3. If equation of plane is ax+by+cz+d=0; then all points-(x1,y1,z1) on the SAME side of it satisfy ax1+by1+cz1+d either all >0 or all <0
	*/
	check := 0
	for i := 0; i < 6; i++ {
		woo1 := Dot2(obj.GetEquations(i), obj.Position)
		woo2 := Dot2(obj.GetEquations(i), point)
		acc1 := obj.GetEquations(i)[3] + Sum(woo1)
		acc2 := obj.GetEquations(i)[3] + Sum(woo2)
		if acc1*acc2 >= 0 {
			//do nothing
		} else {
			check++
		}
	}
	if check == 0 {
		return 1
	} else {
		return 0
	}
}

func (obj Object) PrintObjectData() {
	/*
		What it does:
		1. It stores the plane data of each of the six planes of the object in the global variable 'Data'
		2. The plane data of each plane which is essentially a rectangle consists of the coordinates of each of its 4 corners
		3. This data is finally converted into a json format and printed into the json output file
		4. Plotting of the object boundaries will be done using this data.
	*/
	sign := [][]float64{{1.0, -1.0, -1.0, 1.0}, {1.0, 1.0, -1.0, -1.0}}
	size := []float64{obj.Length / 2.0, obj.Breadth / 2.0, obj.Height / 2.0}
	pos := []float64{obj.Position[0], obj.Position[1], obj.Position[2]}
	for i := 0; i < 3; i++ {
		for j := -1; j <= 1; j = j + 2 {
			t := make([][][]float64, len(Data.Planes)+1)
			copy(t, Data.Planes)
			Data.Planes = t
			var t2 [][]float64
			for ii := 0; ii < 4; ii++ {
				t3 := make([][]float64, len(t2)+1)
				copy(t3, t2)
				t2 = t3
				t4 := []float64{0, 0, 0}
				t4[i] = pos[i] + size[i]*float64(j)
				t4[(i+1)%3] = pos[(i+1)%3] + size[(i+1)%3]*sign[0][ii]
				t4[(i+2)%3] = pos[(i+2)%3] + size[(i+2)%3]*sign[1][ii]
				t2[len(t3)-1] = t4
			}
			Data.Planes[len(t)-1] = t2
		}
	}
}
