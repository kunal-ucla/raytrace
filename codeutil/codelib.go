package codeutil

import "math"

type JsonData struct {
	Receiver    []float64
	Transmitter []float64
	Planes      [][][]float64
	Time        [][]float64
	Points      [][]float64
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
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b
	}
	return c
}

//returns the sum of the elements of an array
//the array should be of type []float64 only and the scalar should be of type float64 only
func Sum(a []float64) float64 {
	var c float64
	for i := 0; i < 3; i++ {
		c = c + a[i]
	}
	return c
}

//returns the dot product of two vectors(arrays)
//the array should be of type []float64 only
func Dot2(a, b []float64) []float64 {
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b[i]
	}
	return c
}

//returns the sum of two vectors(arrays)
//the array should be of type []float64 only
func Sum2(a, b []float64) []float64 {
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] + b[i]
	}
	return c
}

func DistanceBetweenPoints(p1, p2 Point3D) float64 {
	var ans float64
	ans = math.Sqrt(math.Pow(p1[0]-p2[0], 2) + math.Pow(p1[1]-p2[1], 2) + math.Pow(p1[2]-p2[2], 2))
	return ans
}

func GetPOI(ray Ray, obj Object) []float64 {
	t := 0.0
	i_min := -1
	count := 0
	for ii := 0; ii < 6; ii++ {
		acc1 := Sum(Dot2(obj.GetEquations(ii), ray.Point))
		acc2 := Sum(Dot2(obj.GetEquations(ii), ray.Direction))
		tt := -1.0 * (obj.GetEquations(ii)[3] + acc1) / acc2
		if tt > 0 && obj.IsItInside(Sum2(ray.Point, Dot(ray.Direction, tt))) == 1 {
			if count == 0 {
				i_min = ii
				t = tt
			} else if tt < t {
				i_min = ii
				t = tt
			}
			count++
		}
	}
	var ip = []float64{0, 0, 0, 0, 0}
	ip[3] = t
	ip[4] = float64(i_min)
	var safe_ip = []float64{0, 0, 0}
	safe_ip = Sum2(ray.Point, Dot(ray.Direction, t))
	for i := 0; i < 3; i++ {
		ip[i] = safe_ip[i]
	}
	return ip
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
	var ans = []float64{0, 0}
	ans[0] = Sum(Dot2(Sum2(rec.Point, Dot(ray.Point, -1.0)), ray.Direction)) / Sum(Dot2(ray.Direction, ray.Direction))
	p := Sum2(Sum2(ray.Point, Dot(rec.Point, -1.0)), Dot(ray.Direction, ans[0]))
	if math.Sqrt(Sum(Dot2(p, p))) <= rec.Radius && ans[0] > 0.0 {
		ans[1] = 1
	} else {
		ans[1] = 0
	}
	return ans
}

func NextObject(presentIndex int, ray Ray, obstacles []Object) []float64 {
	count := 0
	index := -1
	iii := -1
	t := 0.0
	for i := 0; i < len(obstacles); i++ {
		poi := GetPOI(ray, obstacles[i])
		var poi_4 = []float64{0, 0, 0, 0}
		for poi_i := 0; poi_i < 3; poi_i++ {
			poi_4[poi_i] = poi[poi_i]
		}
		tt := poi[3]
		ii := poi[4]
		if tt > 0.0 {
			if count == 0 {
				index = i
				t = tt
				iii = int(ii)
			} else if presentIndex == i {
				//do nothing
			} else if tt < t {
				index = i
				t = tt
				iii = int(ii)
			}
			count++
		}
	}
	var ans = []float64{0, 0, 0}
	ans[0] = t
	ans[1] = float64(iii)
	ans[2] = float64(index)
	return ans
}

func (obj Object) GetEquations(i int) []float64 {

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

func (obj Object) IsItInside(point Point3D) int {
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

func (obj Object) GetPoints() {

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
