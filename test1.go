package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"time"
)

var plotcode int
var PI float64 = 3.14159265
var fid *os.File

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

func DistanceBetweenPoints(p1, p2 Point3D) float64 {
	var ans float64
	ans = math.Sqrt(math.Pow(p1[0]-p2[0], 2) + math.Pow(p1[1]-p2[1], 2) + math.Pow(p1[2]-p2[2], 2))
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

//multiplies a vector(an array) by a scalar
//the array should be of type []float64 only and the scalar should be of type float64 only
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

func (obj Object) GetPoints() [][]Point3D {

	sign := [][]float64{{1.0, -1.0, -1.0, 1.0}, {1.0, 1.0, -1.0, -1.0}}
	size := []float64{obj.Length / 2.0, obj.Breadth / 2.0, obj.Height / 2.0}
	pos := []float64{obj.Position[0], obj.Position[1], obj.Position[2]}
	for i := 0; i < 3; i++ {
		for j := -1; j <= 1; j = j + 2 {
			t := make([][][]float64, len(data.Planes)+1)
			copy(t, data.Planes)
			data.Planes = t
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
			data.Planes[len(t)-1] = t2
		}
	}
}

func Raytrace(ray Ray, fieldStrength float64, pathLength float64, obstacles []Object, presentIndex int) int {
	var index, iii int
	t := 0.0

	//find the next obstacle in the path of the ray:
	indices := NextObject(-1, ray, obstacles)
	t = indices[0]
	iii = int(indices[1])
	index = int(indices[2])

	if t == 0 {
		return 1 //<----------------------------------!!!!TO DO!!!!---return point where ray dies------------------------------------------------
	}

	//check if it enters receiver region:
	receiverCheck := DoesItPass(ray, receiver)
	didItReach := 0
	if receiverCheck[1] == 1 && receiverCheck[0] < t {
		t = receiverCheck[0]
		didItReach = 1
	}

	//find the point of intersection and set it for next rays:
	p := Sum2(ray.Point, Dot(ray.Direction, t))
	pathLength = pathLength + t*math.Sqrt(Sum(Dot2(ray.Direction, ray.Direction)))
	//var bufferRay Ray
	reflectedRay := Ray{Point: p}
	transmittedRay := Ray{Point: p}
	small_t := 0.01
	pp := Sum2(p, Dot(ray.Direction, small_t))
	nextIndex := 0
	for lo := 0; lo < len(obstacles); lo++ {
		if obstacles[lo].IsItInside(pp) == 1 && lo != 0 {
			nextIndex = lo
		}
	}

	//did it reach?
	if didItReach == 1 {
		timeOfReach := pathLength / 3e8
		{
			t := make([][]float64, len(data.Time)+1)
			copy(t, data.Time)
			data.Time = t
			data.Time[len(t)-1] = []float64{timeOfReach, fieldStrength}
		}
		{
			t := make([][]float64, len(data.Points)+2)
			copy(t, data.Points)
			data.Points = t
			data.Points[len(t)-2] = []float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(plotcode)}
			data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		}
		fmt.Print("|")
		plotcode++
		return 2
	}

	//field attenuation with distance:
	fieldStrength *= math.Exp(-1 * pathLength * 0.4)

	//if field falls below threshold (set as 0.1) then stop
	if fieldStrength < 1e-7 {
		return 1
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---find exact point where the ray dies----------------------------------------

	//set the direction ratios of the reflected ray here:
	acc1 := Sum(Dot2(ray.Direction, ray.Direction))
	acc2 := -1.0 * (Sum(Dot2(ray.Direction, obstacles[index].GetEquations(iii))))
	acc3 := Sum(Dot2(obstacles[index].GetEquations(iii), obstacles[index].GetEquations(iii))) - math.Pow(obstacles[index].GetEquations(iii)[3], 2)
	t = acc2 / acc1
	normal_check1 := Sum2(p, Dot(ray.Direction, small_t))
	normal_check2 := Sum2(p, Dot(obstacles[index].GetEquations(iii), small_t))
	f_normal := -1.0 * IsSameSide(normal_check1, normal_check2, obstacles[index].GetEquations(iii))
	i_dot_n := 2.0 * acc2 // * f_normal;
	ans := Sum2(Dot(obstacles[index].GetEquations(iii), i_dot_n), ray.Direction)
	reflectedRay.Direction = ans

	//set the direction ratios of the transmitted ray here:
	cosTheta1 := -acc2 / math.Sqrt(acc1*acc3)
	sinTheta1 := math.Sin(math.Acos(cosTheta1))
	n := obstacles[presentIndex].R_index / obstacles[nextIndex].R_index
	sinTheta2 := n * sinTheta1
	//cosTheta2 := math.Cos(math.Asin(sinTheta2))
	t_c := -1.0 * math.Sqrt(1-math.Pow(sinTheta2, 2)) / math.Sqrt(acc3)
	lollipop := f_normal * cosTheta1 / math.Sqrt(acc3)
	t_par := Dot(Sum2(Dot(ray.Direction, 1/math.Sqrt(acc1)), Dot(obstacles[index].GetEquations(iii), lollipop)), 2)
	t_per := Dot(obstacles[index].GetEquations(iii), f_normal*t_c)
	t_total := Sum2(t_par, t_per)
	transmittedRay.Direction = t_total

	//trace the reflected and refracted rays next:
	ref_return := 1
	if index == 0 || presentIndex == 0 {
		ref_return = Raytrace(reflectedRay, obstacles[presentIndex].R_coeff*fieldStrength, pathLength, obstacles, presentIndex)
	}
	trans_return := Raytrace(transmittedRay, obstacles[index].T_coeff*fieldStrength, pathLength, obstacles, nextIndex)
	if ref_return*trans_return >= 2 {
		{
			t := make([][]float64, len(data.Points)+2)
			copy(t, data.Points)
			data.Points = t
			data.Points[len(t)-2] = []float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(plotcode)}
			data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		}
		plotcode++
	}

	// ch <- ray
	return ref_return * trans_return
}

type Data struct {
	Receiver    []float64
	Transmitter []float64
	Planes      [][][]float64
	Time        [][]float64
	Points      [][]float64
}

var data Data

var receiver Receiver = Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

var transmitter Transmitter = Transmitter{Point: []float64{-2.0174, -2.58, 0}}

func main() {

	start := time.Now()

	fid, _ = os.Create("out.json")

	data.Receiver = []float64{receiver.Point[0], receiver.Point[1], receiver.Point[2], receiver.Radius}
	data.Transmitter = transmitter.Point

	Room := Object{Length: 13, Breadth: 8.6, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 1, Position: []float64{0, 0, 0}}
	Room.GetPoints()

	Box := Object{Length: 0.03, Breadth: 2.5, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-3.6586, -2.163, 0}}
	Box.GetPoints()

	Box1 := Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, 3.9775, 0}}
	Box1.GetPoints()

	Box2 := Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, 3.9775, 0}}
	Box2.GetPoints()

	Box3 := Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, -3.9775, 0}}
	Box3.GetPoints()

	Box4 := Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, -3.9775, 0}}
	Box4.GetPoints()

	var obstacles []Object = []Object{Room, Box, Box1, Box2, Box3, Box4}

	//receiver = Receiver{point: []float64{-5, -3.08, 0}, radius: 0.2241}
	//move the receiver along a straight line??

	//somehow start many rays from transmitter
	fR := 0.6
	fA := 0.05
	fB := 0.05
	count_rays := 0
	for fi := -fB * float64(int(fR/fB)); fi <= fR; fi = fi + fB {
		fr := math.Sqrt(math.Pow(fR, 2) - math.Pow(fi, 2))
		for fa := 0.0; fa <= 2*PI; fa = fa + fA/fr {
			rayX := Ray{Point: transmitter.Point, Direction: []float64{fr * math.Cos(fa), fr * math.Sin(fa), fi}}
			go func(Ray, chan Ray) {
				Raytrace(rayX, 1, 0, obstacles, 0)
				ch <- rayX
			}(rayX, ch)
			count_rays++
		}
	}

	pinbytes, _ := json.MarshalIndent(data, "", "\t")
	// jd:= json.NewEncoder(fid)
	// json.MarshalIndent(v, prefix, indent)
	fmt.Fprintln(fid, string(pinbytes))
	fid.Close()
	elapsed := time.Since(start)
	fmt.Println("\nProcessed ", count_rays, " rays in ", elapsed)
}
