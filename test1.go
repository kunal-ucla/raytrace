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

type Object struct {
	length, breadth, height   float64
	position                  []float64
	r_coeff, t_coeff, r_index float64
}

type Receiver struct {
	point  []float64
	radius float64
}

type Transmitter struct {
	point []float64
}

type Ray struct {
	point     []float64
	direction []float64
}

func getEquations(i int, obj Object) []float64 {

	var ans []float64
	switch i {
	case 0:
		ans = []float64{0, 0, 1, -obj.position[2] + 0.5*obj.height}
	case 1:
		ans = []float64{0, 0, 1, -obj.position[2] - 0.5*obj.height}
	case 2:
		ans = []float64{0, 1, 0, -obj.position[1] + 0.5*obj.breadth}
	case 3:
		ans = []float64{0, 1, 0, -obj.position[1] - 0.5*obj.breadth}
	case 4:
		ans = []float64{1, 0, 0, -obj.position[0] + 0.5*obj.length}
	case 5:
		ans = []float64{1, 0, 0, -obj.position[0] - 0.5*obj.length}
	}
	return ans
}

//multiplies a vector(an array) by a scalar
//the array should be of type []float64 only and the scalar should be of type float64 only
func dot(a []float64, b float64) []float64 {
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b
	}
	return c
}

//returns the sum of the elements of an array
//the array should be of type []float64 only and the scalar should be of type float64 only
func sum(a []float64) float64 {
	var c float64
	for i := 0; i < 3; i++ {
		c = c + a[i]
	}
	return c
}

//returns the dot product of two vectors(arrays)
//the array should be of type []float64 only
func dot2(a, b []float64) []float64 {
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] * b[i]
	}
	return c
}

//returns the sum of two vectors(arrays)
//the array should be of type []float64 only
func sum2(a, b []float64) []float64 {
	var c = []float64{0, 0, 0}
	for i := 0; i < 3; i++ {
		c[i] = a[i] + b[i]
	}
	return c
}

func isItInside(point []float64, obj Object) int {
	check := 0
	for i := 0; i < 6; i++ {
		woo1 := dot2(getEquations(i, obj), obj.position)
		woo2 := dot2(getEquations(i, obj), point)
		acc1 := getEquations(i, obj)[3] + sum(woo1)
		acc2 := getEquations(i, obj)[3] + sum(woo2)
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

func getPOI(ray Ray, obj Object) []float64 {
	t := 0.0
	i_min := -1
	count := 0
	for ii := 0; ii < 6; ii++ {
		acc1 := sum(dot2(getEquations(ii, obj), ray.point))
		acc2 := sum(dot2(getEquations(ii, obj), ray.direction))
		tt := -1.0 * (getEquations(ii, obj)[3] + acc1) / acc2
		if tt > 0 && isItInside(sum2(ray.point, dot(ray.direction, tt)), obj) == 1 {
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
	safe_ip = sum2(ray.point, dot(ray.direction, t))
	for i := 0; i < 3; i++ {
		ip[i] = safe_ip[i]
	}
	return ip
}

func isSameSide(p1 []float64, p2 []float64, plane []float64) float64 {
	check := (plane[0]*p1[0] + plane[1]*p1[1] + plane[2]*p1[2] + plane[3]) * (plane[0]*p2[0] + plane[1]*p2[1] + plane[2]*p2[2] + plane[3])
	if check > 0 {
		return 1.0
	} else {
		return -1.0
	}
}

func doesItPass(ray Ray, rec Receiver) []float64 {
	var ans = []float64{0, 0}
	ans[0] = sum(dot2(sum2(rec.point, dot(ray.point, -1.0)), ray.direction)) / sum(dot2(ray.direction, ray.direction))
	p := sum2(sum2(ray.point, dot(rec.point, -1.0)), dot(ray.direction, ans[0]))
	if math.Sqrt(sum(dot2(p, p))) <= rec.radius && ans[0] > 0.0 {
		ans[1] = 1
	} else {
		ans[1] = 0
	}
	return ans
}

func nextObject(presentIndex int, ray Ray, obstacles []Object) []float64 {
	count := 0
	index := -1
	iii := -1
	t := 0.0
	for i := 0; i < len(obstacles); i++ {
		poi := getPOI(ray, obstacles[i])
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

var receiver Receiver = Receiver{point: []float64{-5, -3.08, 0}, radius: 0.2241}

var transmitter Transmitter = Transmitter{point: []float64{-2.0174, -2.58, 0}}

func getPoints(obj Object) {

	sign := [][]float64{{1.0, -1.0, -1.0, 1.0}, {1.0, 1.0, -1.0, -1.0}}
	size := []float64{obj.length / 2.0, obj.breadth / 2.0, obj.height / 2.0}
	pos := []float64{obj.position[0], obj.position[1], obj.position[2]}
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

func raytrace(ray Ray, fieldStrength float64, pathLength float64, obstacles []Object, presentIndex int) int {
	var index, iii int
	t := 0.0

	//find the next obstacle in the path of the ray:
	indices := nextObject(-1, ray, obstacles)
	t = indices[0]
	iii = int(indices[1])
	index = int(indices[2])

	if t == 0 {
		return 1 //<----------------------------------!!!!TO DO!!!!---return point where ray dies------------------------------------------------
	}

	//check if it enters receiver region:
	receiverCheck := doesItPass(ray, receiver)
	didItReach := 0
	if receiverCheck[1] == 1 && receiverCheck[0] < t {
		t = receiverCheck[0]
		didItReach = 1
	}

	//find the point of intersection and set it for next rays:
	p := sum2(ray.point, dot(ray.direction, t))
	pathLength = pathLength + t*math.Sqrt(sum(dot2(ray.direction, ray.direction)))
	//var bufferRay Ray
	reflectedRay := Ray{point: p}
	transmittedRay := Ray{point: p}
	small_t := 0.01
	pp := sum2(p, dot(ray.direction, small_t))
	nextIndex := 0
	for lo := 0; lo < len(obstacles); lo++ {
		if isItInside(pp, obstacles[lo]) == 1 && lo != 0 {
			nextIndex = lo
		}
	}
	// ppp := sum2(p, dot(ray.direction, -small_t))
	// prevIndex := 0
	// for lo := 0; lo < len(obstacles); lo++ {
	// 	if isItInside(pp, obstacles[lo]) == 1 && lo != 0 {
	// 		prevIndex = lo
	// 	}
	// }

	//did it reach?
	if didItReach == 1 {
		//fmt.Print("reached!")
		timeOfReach := pathLength / 3e8
		//fmt.Print(pathLength, "\n")
		//fmt.Fprint(fid, "Time ", timeOfReach, " ", fieldStrength, "\n")
		{
			t := make([][]float64, len(data.Time)+1)
			copy(t, data.Time)
			data.Time = t
			data.Time[len(t)-1] = []float64{timeOfReach, fieldStrength}
		}
		//fmt.Fprint(fid, ray.point[0], " ", ray.point[1], " ", ray.point[2], " ", plotcode, "\n")
		{
			t := make([][]float64, len(data.Points)+2)
			copy(t, data.Points)
			data.Points = t
			data.Points[len(t)-2] = []float64{ray.point[0], ray.point[1], ray.point[2], float64(plotcode)}
			data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		}
		//fmt.Fprint(fid, p[0], " ", p[1], " ", p[2], " ", plotcode, "\n")
		fmt.Print("|")
		plotcode++
		return 2
	}

	//field attenuation with distance:
	fieldStrength *= math.Exp(-1 * pathLength * 0.05)

	//if field falls below threshold (set as 0.1) then stop
	if fieldStrength < 0.01 {
		return 1
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^!!!!TO DO!!!!---find exact point where the ray dies----------------------------------------

	//set the direction ratios of the reflected ray here:
	acc1 := sum(dot2(ray.direction, ray.direction))
	acc2 := -1.0 * (sum(dot2(ray.direction, getEquations(iii, obstacles[index]))))
	acc3 := sum(dot2(getEquations(iii, obstacles[index]), getEquations(iii, obstacles[index]))) - math.Pow(getEquations(iii, obstacles[index])[3], 2)
	t = acc2 / acc1
	normal_check1 := sum2(p, dot(ray.direction, small_t))
	normal_check2 := sum2(p, dot(getEquations(iii, obstacles[index]), small_t))
	f_normal := -1.0 * isSameSide(normal_check1, normal_check2, getEquations(iii, obstacles[index]))
	i_dot_n := 2.0 * acc2 // * f_normal;
	ans := sum2(dot(getEquations(iii, obstacles[index]), i_dot_n), ray.direction)
	reflectedRay.direction = ans

	//set the direction ratios of the transmitted ray here:
	cosTheta1 := -acc2 / math.Sqrt(acc1*acc3)
	sinTheta1 := math.Sin(math.Acos(cosTheta1))
	n := obstacles[presentIndex].r_index / obstacles[nextIndex].r_index
	sinTheta2 := n * sinTheta1
	//cosTheta2 := math.Cos(math.Asin(sinTheta2))
	t_c := -1.0 * math.Sqrt(1-math.Pow(sinTheta2, 2)) / math.Sqrt(acc3)
	lollipop := f_normal * cosTheta1 / math.Sqrt(acc3)
	t_par := dot(sum2(dot(ray.direction, 1/math.Sqrt(acc1)), dot(getEquations(iii, obstacles[index]), lollipop)), 2)
	t_per := dot(getEquations(iii, obstacles[index]), f_normal*t_c)
	t_total := sum2(t_par, t_per)
	transmittedRay.direction = t_total

	//trace the reflected and refracted rays next:
	ref_return := 1
	if index == 0 || presentIndex == 0 {
		ref_return = raytrace(reflectedRay, obstacles[presentIndex].r_coeff*fieldStrength, pathLength, obstacles, presentIndex)
	}
	trans_return := raytrace(transmittedRay, obstacles[index].t_coeff*fieldStrength, pathLength, obstacles, nextIndex)
	if ref_return*trans_return >= 2 {
		//fmt.Fprint(fid, ray.point[0], " ", ray.point[1], " ", ray.point[2], " ", plotcode, "\n")
		//fmt.Fprint(fid, p[0], " ", p[1], " ", p[2], " ", plotcode, "\n")
		{
			t := make([][]float64, len(data.Points)+2)
			copy(t, data.Points)
			data.Points = t
			data.Points[len(t)-2] = []float64{ray.point[0], ray.point[1], ray.point[2], float64(plotcode)}
			data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		}
		plotcode++
	}
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

func main() {

	start := time.Now()

	fid, _ = os.Create("out.json")

	//fmt.Fprint(fid, "Receiver ", receiver.point[0], " ", receiver.point[1], " ", receiver.point[2], receiver.radius, "\n")
	//fmt.Fprint(fid, "Transmitter ", transmitter.point[0], " ", transmitter.point[1], " ", transmitter.point[2], "\n")
	data.Receiver = []float64{receiver.point[0], receiver.point[1], receiver.point[2], receiver.radius}
	data.Transmitter = transmitter.point

	Room := Object{length: 13, breadth: 8.6, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 1, position: []float64{0, 0, 0}}
	getPoints(Room)

	Box := Object{length: 0.03, breadth: 2.5, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 2, position: []float64{-3.6586, -2.163, 0}}
	getPoints(Box)

	Box1 := Object{length: 0.91, breadth: 0.645, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 2, position: []float64{6.045, 3.9775, 0}}
	getPoints(Box1)

	Box2 := Object{length: 0.91, breadth: 0.645, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 2, position: []float64{-6.045, 3.9775, 0}}
	getPoints(Box2)

	Box3 := Object{length: 0.91, breadth: 0.645, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 2, position: []float64{6.045, -3.9775, 0}}
	getPoints(Box3)

	Box4 := Object{length: 0.91, breadth: 0.645, height: 3, r_coeff: 0.4, t_coeff: 0, r_index: 2, position: []float64{-6.045, -3.9775, 0}}
	getPoints(Box4)

	var obstacles []Object = []Object{Room, Box, Box1, Box2, Box3, Box4}

	//receiver = Receiver{point: []float64{-5, -3.08, 0}, radius: 0.2241}
	//move the receiver along a straight line??

	//somehow start many rays from transmitter
	fR := 0.6
	fA := 0.005
	fB := 0.005
	count_rays := 0
	for fi := -fB * float64(int(fR/fB)); fi <= fR; fi = fi + fB {
		fr := math.Sqrt(math.Pow(fR, 2) - math.Pow(fi, 2))
		for fa := 0.0; fa <= 2*PI; fa = fa + fA/fr {
			rayX := Ray{point: transmitter.point, direction: []float64{fr * math.Cos(fa), fr * math.Sin(fa), fi}}
			raytrace(rayX, 1, 0, obstacles, 0)
			count_rays++
		}
	}

	pinbytes, _ := json.Marshal(data)

	fmt.Fprintln(fid, string(pinbytes))
	fid.Close()
	elapsed := time.Since(start)
	fmt.Println("\nProcessed ", count_rays, " rays in ", elapsed)
}
