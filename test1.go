package main

import (
	"fmt"
	"math"
	"os"
	"sync"
	"time"

	"github.com/user/codeutil"
)

type RayLog struct {
	TimeOfReach   float64
	FieldStrength float64
	Points        []codeutil.Point3D
	SegmentId     int
}

func Raytrace(rayid int, ray codeutil.Ray, fieldStrength float64, pathLength float64, obstacles []codeutil.Object, presentIndex int, ch chan RayLog) int {
	var index, iii int
	t := 0.0

	//find the next obstacle in the path of the ray:
	indices := codeutil.NextObject(-1, ray, obstacles)
	t = indices[0]
	iii = int(indices[1])
	index = int(indices[2])

	if t == 0 {
		return 1 //<----------------------------------!!!!TO DO!!!!---return point where ray dies------------------------------------------------
	}
	var receiverLocal codeutil.Receiver = codeutil.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}
	//check if it enters receiver region:
	receiverCheck := codeutil.DoesItPass(ray, receiverLocal)
	didItReach := 0
	if receiverCheck[1] == 1 && receiverCheck[0] < t {
		t = receiverCheck[0]
		didItReach = 1
	}

	//find the point of intersection and set it for next rays:
	p := codeutil.Sum2(ray.Point, codeutil.Dot(ray.Direction, t))
	pathLength = pathLength + t*math.Sqrt(codeutil.Sum(codeutil.Dot2(ray.Direction, ray.Direction)))
	//var bufferRay Ray
	reflectedRay := codeutil.Ray{Point: p}
	transmittedRay := codeutil.Ray{Point: p}
	small_t := 0.01
	pp := codeutil.Sum2(p, codeutil.Dot(ray.Direction, small_t))
	nextIndex := 0
	for lo := 0; lo < len(obstacles); lo++ {
		if obstacles[lo].IsItInside(pp) == 1 && lo != 0 {
			nextIndex = lo
		}
	}
	var localLog RayLog
	//did it reach?
	if didItReach == 1 {
		timeOfReach := pathLength / 3e8
		// {
		// 	t := make([][]float64, len(codeutil.Data.Time)+1)
		// 	copy(t, codeutil.Data.Time)
		// 	codeutil.Data.Time = t
		// 	codeutil.Data.Time[len(t)-1] = []float64{timeOfReach, fieldStrength}
		// }
		// {
		// 	t := make([][]float64, len(codeutil.Data.Points)+2)
		// 	copy(t, codeutil.Data.Points)
		// 	codeutil.Data.Points = t
		// 	codeutil.Data.Points[len(t)-2] = []float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(plotcode)}
		// 	codeutil.Data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		// }
		localLog.FieldStrength = fieldStrength
		localLog.TimeOfReach = timeOfReach
		localLog.Points = []codeutil.Point3D{ray.Point, p}
		//localLog.SegmentId = plotcode
		ch <- localLog
		//plotcode++
		fmt.Print("|")
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
	acc1 := codeutil.Sum(codeutil.Dot2(ray.Direction, ray.Direction))
	acc2 := -1.0 * (codeutil.Sum(codeutil.Dot2(ray.Direction, obstacles[index].GetEquations(iii))))
	acc3 := codeutil.Sum(codeutil.Dot2(obstacles[index].GetEquations(iii), obstacles[index].GetEquations(iii))) - math.Pow(obstacles[index].GetEquations(iii)[3], 2)
	t = acc2 / acc1
	normal_check1 := codeutil.Sum2(p, codeutil.Dot(ray.Direction, small_t))
	normal_check2 := codeutil.Sum2(p, codeutil.Dot(obstacles[index].GetEquations(iii), small_t))
	f_normal := -1.0 * codeutil.IsSameSide(normal_check1, normal_check2, obstacles[index].GetEquations(iii))
	i_dot_n := 2.0 * acc2 // * f_normal;
	ans := codeutil.Sum2(codeutil.Dot(obstacles[index].GetEquations(iii), i_dot_n), ray.Direction)
	reflectedRay.Direction = ans

	//set the direction ratios of the transmitted ray here:
	cosTheta1 := -acc2 / math.Sqrt(acc1*acc3)
	sinTheta1 := math.Sin(math.Acos(cosTheta1))
	n := obstacles[presentIndex].R_index / obstacles[nextIndex].R_index
	sinTheta2 := n * sinTheta1
	//cosTheta2 := math.Cos(math.Asin(sinTheta2))
	t_c := -1.0 * math.Sqrt(1-math.Pow(sinTheta2, 2)) / math.Sqrt(acc3)
	lollipop := f_normal * cosTheta1 / math.Sqrt(acc3)
	t_par := codeutil.Dot(codeutil.Sum2(codeutil.Dot(ray.Direction, 1/math.Sqrt(acc1)), codeutil.Dot(obstacles[index].GetEquations(iii), lollipop)), 2)
	t_per := codeutil.Dot(obstacles[index].GetEquations(iii), f_normal*t_c)
	t_total := codeutil.Sum2(t_par, t_per)
	transmittedRay.Direction = t_total

	//trace the reflected and refracted rays next:
	ref_return := 1
	if index == 0 || presentIndex == 0 {
		ref_return = Raytrace(rayid, reflectedRay, obstacles[presentIndex].R_coeff*fieldStrength, pathLength, obstacles, presentIndex, ch)
	}
	trans_return := Raytrace(rayid, transmittedRay, obstacles[index].T_coeff*fieldStrength, pathLength, obstacles, nextIndex, ch)
	if ref_return*trans_return >= 2 {
		// {
		// 	t := make([][]float64, len(codeutil.Data.Points)+2)
		// 	copy(t, codeutil.Data.Points)
		// 	codeutil.Data.Points = t
		// 	codeutil.Data.Points[len(t)-2] = []float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(plotcode)}
		// 	codeutil.Data.Points[len(t)-1] = []float64{p[0], p[1], p[2], float64(plotcode)}
		// }
		// localLog.Points = []codeutil.Point3D{ray.Point, p}
		// localLog.SegmentId = plotcode
		// plotcode++
		//ch <- localLog
	}
	return ref_return * trans_return
}

var fid *os.File

var PI float64 = 3.14159265

var plotcode int

var receiver codeutil.Receiver = codeutil.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

var transmitter codeutil.Transmitter = codeutil.Transmitter{Point: []float64{-2.0174, -2.58, 0}}

// func init() {
// 	runtime.GOMAXPROCS(runtime.NumCPU())
// }

func main() {

	start := time.Now()

	fid, _ = os.Create("out.json")

	codeutil.Data.Receiver = []float64{receiver.Point[0], receiver.Point[1], receiver.Point[2], receiver.Radius}
	codeutil.Data.Transmitter = transmitter.Point

	Room := codeutil.Object{Length: 13, Breadth: 8.6, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 1, Position: []float64{0, 0, 0}}
	Room.GetPoints()

	Box := codeutil.Object{Length: 0.03, Breadth: 2.5, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-3.6586, -2.163, 0}}
	Box.GetPoints()

	Box1 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, 3.9775, 0}}
	Box1.GetPoints()

	Box2 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, 3.9775, 0}}
	Box2.GetPoints()

	Box3 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, -3.9775, 0}}
	Box3.GetPoints()

	Box4 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, -3.9775, 0}}
	Box4.GetPoints()

	var obstacles []codeutil.Object = []codeutil.Object{Room, Box, Box1, Box2, Box3, Box4}

	//receiver = codeutil.Receiver{point: []float64{-5, -3.08, 0}, radius: 0.2241}
	//move the receiver along a straight line??

	//somehow start many rays from transmitter
	fR := 0.6
	fA := 0.005
	fB := 0.005
	count_rays := 0

	ch := make(chan RayLog, 100000)

	go func() {
		for {
			x, ok := <-ch
			if !ok {
				close(ch)
				break
			}
			fmt.Println(x)
		}
	}()

	var wg sync.WaitGroup
	for fi := -fB * float64(int(fR/fB)); fi <= fR; fi = fi + fB {
		fr := math.Sqrt(math.Pow(fR, 2) - math.Pow(fi, 2))
		for fa := 0.0; fa <= 2*PI; fa = fa + fA/fr {
			rayX := codeutil.Ray{Point: transmitter.Point, Direction: []float64{fr * math.Cos(fa), fr * math.Sin(fa), fi}}
			wg.Add(1)
			go func(rayX codeutil.Ray, count_rays int) {
				fmt.Println(count_rays)
				Raytrace(count_rays, rayX, 1, 0, obstacles, 0, ch)
				wg.Done()
			}(rayX, count_rays)
			count_rays++
		}
	}

	wg.Wait()

	// var input string
	// fmt.Scanln(&input)

	//pinbytes, _ := json.MarshalIndent(codeutil.Data, "", "\t")
	// jd:= json.NewEncoder(fid)
	// json.MarshalIndent(v, prefix, indent)
	//fmt.Fprintln(fid, string(pinbytes))
	fid.Close()
	elapsed := time.Since(start)
	fmt.Println("\nProcessed ", count_rays, " rays in ", elapsed)
	//fmt.Println("cpus num: ", runtime.NumCPU())
}
