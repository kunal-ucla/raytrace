package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"runtime"
	"strconv"
	"sync"
	"time"

	"github.com/user/codeutil"
)

type RayLog struct {
	Time       []float64
	Points     []codeutil.Point3D
	DidItReach int
}

func Raytrace(rayid int, ray codeutil.Ray, fieldStrength float64, pathLength float64, obstacles []codeutil.Object, presentIndex int, ch chan RayLog) int {

	/*Find the next obstacle in the path of the ray:*/
	t, next_object_index, next_plane_index := codeutil.NextObject(-1, ray, obstacles)
	/*
		t : the 't' parameter of the next point at which the ray is going to reflect/transmit
		next_object_index : the index of the object which the ray is going to meet along its path
		next_plane_index : the index of the plane of the above object that the ray is going to land on
	*/

	if t == 0 {
		return 1
		/*
			This is the case when the ray does not intersect any object at all
			This will happen with the ray that gets transmitted through the boundaries of the room and gets outside.
			Because we are not interested in tracing the rays that go outside our room boundaries, we stop the ray here by returning 1.
		*/
	}

	/*Here we create a local instance of the receiver because the goroutine should not be depending on a global variable to increase efficiency of the concurrency.*/
	var receiverLocal codeutil.Receiver = codeutil.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

	/*Check if the ray enters the receiver region:*/
	receiverCheck := codeutil.DoesItPass(ray, receiverLocal)

	/*Initialize a variable(to 0) named 'didItReach' which will tell us whether the ray reaches the receiver or not*/
	didItReach := 0

	/*
		Now we have to make sure that it does not encounter any object before it reaches(if it does) the receiver region.
		For that, we check compare the 't' parameter of the next intersection point that we initially stored in 't' and the 't' parameter of the point ...
		... that is closest to the receiver which is given by the second element returned by codeutil.DoesItPass() stored in receiverCheck[1]
		Finally set the didItReach variable as 1 if it reached the receiver.
	*/
	if receiverCheck[1] == 1 && receiverCheck[0] < t {
		t = receiverCheck[0]
		didItReach = 1
	}

	/*Determine the point of intersection and store it in an array named 'nextPoint'*/
	nextPoint := codeutil.Sum2(ray.Point, codeutil.Dot(ray.Direction, t))

	/*Update the pathlength of the ray by adding the distance between the present point and the ext point*/
	pathLength = pathLength + t*math.Sqrt(codeutil.Sum(codeutil.Dot2(ray.Direction, ray.Direction)))

	/*Set the point of origin of the reflected and transmitted rays. The direction will be set later*/
	reflectedRay := codeutil.Ray{Point: nextPoint}
	transmittedRay := codeutil.Ray{Point: nextPoint}

	/*
		Set a variable called small_t which will be used to obtain a point just above the next point in the direction of the present ray
		The point thus obtained will tell us in which object the next transmitted ray is going to travel.
		Knowing the object has two important uses--1-the angle of deflection of the transmitted ray depends on the propertes of next medium ...
		... --2-we have to set the parameter 'presentIndex' of the next raytrace function before calling it.
	*/
	small_t := 0.01
	pp := codeutil.Sum2(nextPoint, codeutil.Dot(ray.Direction, small_t))
	nextIndex := 0
	for lo := 0; lo < len(obstacles); lo++ {
		if obstacles[lo].DoesItContain(pp) == 1 && lo != 0 {
			nextIndex = lo
		}
	}

	var localLog RayLog
	/*Did it reach?*/
	if didItReach == 1 {
		timeOfReach := pathLength / 3e8
		localLog.Time = []float64{timeOfReach, fieldStrength}
		localLog.Points = []codeutil.Point3D{[]float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(rayid)}, []float64{nextPoint[0], nextPoint[1], nextPoint[2], float64(rayid)}}
		localLog.DidItReach = didItReach
		ch <- localLog
		rayid++
		fmt.Print("|")
		return 2
	}

	/*Field attenuation with distance:*/
	fieldStrength *= math.Exp(-1 * pathLength * 0.4)

	/*If field falls below threshold (set as 0.1) then stop*/
	if fieldStrength < 1e-7 {
		return 1
	}

	/*Set the direction ratios of the reflected ray here:*/
	acc1 := codeutil.Sum(codeutil.Dot2(ray.Direction, ray.Direction))
	acc2 := -1.0 * (codeutil.Sum(codeutil.Dot2(ray.Direction, obstacles[next_object_index].GetEquations(next_plane_index))))
	acc3 := codeutil.Sum(codeutil.Dot2(obstacles[next_object_index].GetEquations(next_plane_index), obstacles[next_object_index].GetEquations(next_plane_index))) - math.Pow(obstacles[next_object_index].GetEquations(next_plane_index)[3], 2)
	t = acc2 / acc1
	normal_check1 := codeutil.Sum2(nextPoint, codeutil.Dot(ray.Direction, small_t))
	normal_check2 := codeutil.Sum2(nextPoint, codeutil.Dot(obstacles[next_object_index].GetEquations(next_plane_index), small_t))
	f_normal := -1.0 * codeutil.IsSameSide(normal_check1, normal_check2, obstacles[next_object_index].GetEquations(next_plane_index))
	i_dot_n := 2.0 * acc2 // * f_normal;
	ans := codeutil.Sum2(codeutil.Dot(obstacles[next_object_index].GetEquations(next_plane_index), i_dot_n), ray.Direction)
	reflectedRay.Direction = ans

	/*Set the direction ratios of the transmitted ray here:*/
	cosTheta1 := -acc2 / math.Sqrt(acc1*acc3)
	sinTheta1 := math.Sin(math.Acos(cosTheta1))
	n := obstacles[presentIndex].R_index / obstacles[nextIndex].R_index
	sinTheta2 := n * sinTheta1
	t_c := -1.0 * math.Sqrt(1-math.Pow(sinTheta2, 2)) / math.Sqrt(acc3)
	lollipop := f_normal * cosTheta1 / math.Sqrt(acc3)
	t_par := codeutil.Dot(codeutil.Sum2(codeutil.Dot(ray.Direction, 1/math.Sqrt(acc1)), codeutil.Dot(obstacles[next_object_index].GetEquations(next_plane_index), lollipop)), 2)
	t_per := codeutil.Dot(obstacles[next_object_index].GetEquations(next_plane_index), f_normal*t_c)
	t_total := codeutil.Sum2(t_par, t_per)
	transmittedRay.Direction = t_total

	/*Trace the reflected and refracted rays next:*/
	ref_return := 1
	if next_object_index == 0 || presentIndex == 0 {
		ref_return = Raytrace(rayid, reflectedRay, obstacles[presentIndex].R_coeff*fieldStrength, pathLength, obstacles, presentIndex, ch)
	}
	trans_return := Raytrace(rayid, transmittedRay, obstacles[next_object_index].T_coeff*fieldStrength, pathLength, obstacles, nextIndex, ch)
	if ref_return*trans_return >= 2 {
		localLog.DidItReach = didItReach
		localLog.Points = []codeutil.Point3D{[]float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(rayid)}, []float64{nextPoint[0], nextPoint[1], nextPoint[2], float64(rayid)}}
		ch <- localLog
		rayid++
	}
	return ref_return * trans_return
}

var fid *os.File

var PI float64 = 3.14159265

var receiver codeutil.Receiver = codeutil.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

var transmitter codeutil.Transmitter = codeutil.Transmitter{Point: []float64{-2.0174, -2.58, 0}}

func main() {

	numCores, _ := strconv.Atoi(os.Args[1])
	runtime.GOMAXPROCS(numCores)

	start := time.Now()

	fid, _ = os.Create("out_" + os.Args[1] + "_cores.json")

	codeutil.Data.Receiver = []float64{receiver.Point[0], receiver.Point[1], receiver.Point[2], receiver.Radius}
	codeutil.Data.Transmitter = transmitter.Point

	Room := codeutil.Object{Length: 13, Breadth: 8.6, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 1, Position: []float64{0, 0, 0}}
	Room.PrintObjectData()

	Box := codeutil.Object{Length: 0.03, Breadth: 2.5, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-3.6586, -2.163, 0}}
	Box.PrintObjectData()

	Box1 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, 3.9775, 0}}
	Box1.PrintObjectData()

	Box2 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, 3.9775, 0}}
	Box2.PrintObjectData()

	Box3 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, -3.9775, 0}}
	Box3.PrintObjectData()

	Box4 := codeutil.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, -3.9775, 0}}
	Box4.PrintObjectData()

	var obstacles []codeutil.Object = []codeutil.Object{Room, Box, Box1, Box2, Box3, Box4}

	/*Initiate the rays from transmitter*/
	fR := 0.6
	fA := 0.05
	fB := 0.05
	count_rays := 0

	ch := make(chan RayLog, 100000)

	go func() {
		for {
			x, ok := <-ch
			if !ok {
				close(ch)
				break
			}
			{
				t := make([][]float64, len(codeutil.Data.Points)+2)
				copy(t, codeutil.Data.Points)
				codeutil.Data.Points = t
				codeutil.Data.Points[len(t)-2] = x.Points[0]
				codeutil.Data.Points[len(t)-1] = x.Points[1]
			}
			if x.DidItReach == 1 {
				t := make([][]float64, len(codeutil.Data.Time)+1)
				copy(t, codeutil.Data.Time)
				codeutil.Data.Time = t
				codeutil.Data.Time[len(t)-1] = x.Time
			}
		}
	}()

	var wg sync.WaitGroup

	for fi := -fB * float64(int(fR/fB)); fi <= fR; fi = fi + fB {
		fr := math.Sqrt(math.Pow(fR, 2) - math.Pow(fi, 2))
		for fa := 0.0; fa <= 2*PI; fa = fa + fA/fr {
			rayX := codeutil.Ray{Point: transmitter.Point, Direction: []float64{fr * math.Cos(fa), fr * math.Sin(fa), fi}}
			wg.Add(1)
			go func(rayX codeutil.Ray, count_rays int) {
				Raytrace(count_rays, rayX, 1, 0, obstacles, 0, ch)
				wg.Done()
			}(rayX, count_rays)
			count_rays++
		}
	}

	wg.Wait()

	elapsed := time.Since(start)
	fmt.Println("\nDone!!!\nProcessed ", count_rays, " rays in ", elapsed)
	codeutil.Data.Process.TimeTaken = float64(elapsed)
	codeutil.Data.Process.NumCores = int(numCores)

	/*Print the sotred data into the out.json file*/
	pinbytes, _ := json.MarshalIndent(codeutil.Data, "", "\t")
	jd := json.NewEncoder(fid)
	json.MarshalIndent(jd, "", "\t")
	fmt.Fprintln(fid, string(pinbytes))
	fid.Close()
}
