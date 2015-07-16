package main

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"runtime"
	"sync"
	"time"
	"raytracing"
)

type RayLog struct {
	Time       []float64
	Points     []rt.Point3D
	DidItReach int
}

func Raytrace(rayid *int, ray rt.Ray, fieldStrength float64, pathLength float64, obstacles []rt.Object, presentIndex int, ch chan RayLog) int {

	/*Find the next obstacle in the path of the ray:*/
	t, next_object_index, next_plane_index := rt.NextObject(-1, ray, obstacles)
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
	var receiverLocal rt.Receiver = rt.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

	/*Check if the ray enters the receiver region:*/
	receiverCheck := rt.DoesItPass(ray, receiverLocal)

	/*Initialize a variable(to 0) named 'didItReach' which will tell us whether the ray reaches the receiver or not*/
	didItReach := 0

	/*
		Now we have to make sure that it does not encounter any object before it reaches(if it does) the receiver region.
		For that, we check compare the 't' parameter of the next intersection point that we initially stored in 't' and the 't' parameter of the point ...
		... that is closest to the receiver which is given by the second element returned by rt.DoesItPass() stored in receiverCheck[1]
		Finally set the didItReach variable as 1 if it reached the receiver.
	*/
	if receiverCheck[1] == 1 && receiverCheck[0] < t {
		t = receiverCheck[0]
		didItReach = 1
	}

	/*Determine the point of intersection and store it in an array named 'nextPoint'*/
	nextPoint := rt.Sum2(ray.Point, rt.Dot(ray.Direction, t))

	/*Update the pathlength of the ray by adding the distance between the present point and the ext point*/
	pathLength = pathLength + t*math.Sqrt(rt.Sum(rt.Dot2(ray.Direction, ray.Direction)))

	/*Set the point of origin of the reflected and transmitted rays. The direction will be set later*/
	reflectedRay := rt.Ray{Point: nextPoint}
	transmittedRay := rt.Ray{Point: nextPoint}

	/*
		Set a variable called small_t which will be used to obtain a point just above the next point in the direction of the present ray
		The point thus obtained will tell us in which object the next transmitted ray is going to travel.
		Knowing the object has two important uses--1-the angle of deflection of the transmitted ray depends on the propertes of next medium ...
		... --2-we have to set the parameter 'presentIndex' of the next raytrace function before calling it.
	*/
	small_t := 0.01
	pp := rt.Sum2(nextPoint, rt.Dot(ray.Direction, small_t))
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
		localLog.Points = []rt.Point3D{[]float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(*rayid)}, []float64{nextPoint[0], nextPoint[1], nextPoint[2], float64(*rayid)}}
		localLog.DidItReach = didItReach
		*rayid = *(rayid) + 1
		ch <- localLog
		fmt.Print("|")
		return 2
	}

	/*Field attenuation with distance:*/
	fieldStrength *= math.Exp(-1 * pathLength * 0.4)

	/*If field falls below threshold (set as 0.1) then stop*/
	if fieldStrength < 1e-7 {
		return 1
	}

	/*Get the direction of reflected and the transmitted rays*/
	reflectedRay.Direction, transmittedRay.Direction = rt.GetNextRaysDirection(ray, obstacles, presentIndex, nextIndex, next_object_index, next_plane_index, small_t, nextPoint)

	/*Trace the reflected and refracted rays next:*/
	ref_return := 1
	if next_object_index == 0 || presentIndex == 0 {
		/*The purpose of if condition here: Reflection should take place only if the ray is hitting on walls of room OR the OUTER surface of an object*/
		/*Ray travelling in the room medium(and not inside any object) would imply that presentIndex = 0*/
		/*OR maybe that the ray is travelling inside an object which is touching one of the walls of the room; and the ray is hitting that wall; => next_object_index = 0*/
		ref_return = Raytrace(rayid, reflectedRay, obstacles[presentIndex].R_coeff*fieldStrength, pathLength, obstacles, presentIndex, ch)
	}
	trans_return := Raytrace(rayid, transmittedRay, obstacles[next_object_index].T_coeff*fieldStrength, pathLength, obstacles, nextIndex, ch)

	if ref_return*trans_return >= 2 {
		localLog.DidItReach = didItReach
		localLog.Points = []rt.Point3D{[]float64{ray.Point[0], ray.Point[1], ray.Point[2], float64(*rayid)}, []float64{nextPoint[0], nextPoint[1], nextPoint[2], float64(*rayid)}}
		*rayid = *(rayid) + 1
		ch <- localLog
	}
	return ref_return * trans_return
}

var fid *os.File

var PI float64 = 3.14159265

var receiver rt.Receiver = rt.Receiver{Point: []float64{-5, -3.08, 0}, Radius: 0.2241}

var transmitter rt.Transmitter = rt.Transmitter{Point: []float64{-2.0174, -2.58, 0}}

func main() {

	//numCores, _ := strconv.Atoi(os.Args[1])
	runtime.GOMAXPROCS(2)

	start := time.Now()

	fid, _ = os.Create("out_2_cores.json")

	/*Create objects including the room itself*/
	rt.Data.Receiver = []float64{receiver.Point[0], receiver.Point[1], receiver.Point[2], receiver.Radius}
	rt.Data.Transmitter = transmitter.Point

	Room := rt.Object{Length: 13, Breadth: 8.6, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 1, Position: []float64{0, 0, 0}}
	Room.PrintObjectData()

	Box := rt.Object{Length: 0.03, Breadth: 2.5, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-3.6586, -2.163, 0}}
	Box.PrintObjectData()

	Box1 := rt.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, 3.9775, 0}}
	Box1.PrintObjectData()

	Box2 := rt.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, 3.9775, 0}}
	Box2.PrintObjectData()

	Box3 := rt.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{6.045, -3.9775, 0}}
	Box3.PrintObjectData()

	Box4 := rt.Object{Length: 0.91, Breadth: 0.645, Height: 3, R_coeff: 0.4, T_coeff: 0, R_index: 2, Position: []float64{-6.045, -3.9775, 0}}
	Box4.PrintObjectData()

	var obstacles []rt.Object = []rt.Object{Room, Box, Box1, Box2, Box3, Box4}

	/*Initiate the rays from transmitter*/
	fR := 0.6
	fA := 0.005
	fB := 0.005
	count_rays := 0
	rayid := 0

	ch := make(chan RayLog, 100000)

	go func() {
		for {
			x, ok := <-ch
			if !ok {
				close(ch)
				break
			}
			{
				t := make([][]float64, len(rt.Data.Points)+2)
				copy(t, rt.Data.Points)
				rt.Data.Points = t
				rt.Data.Points[len(t)-2] = x.Points[0]
				rt.Data.Points[len(t)-1] = x.Points[1]
			}
			if x.DidItReach == 1 {
				t := make([][]float64, len(rt.Data.Time)+1)
				copy(t, rt.Data.Time)
				rt.Data.Time = t
				rt.Data.Time[len(t)-1] = x.Time
			}
		}
	}()

	var wg sync.WaitGroup

	for fi := -fB * float64(int(fR/fB)); fi <= fR; fi = fi + fB {
		fr := math.Sqrt(math.Pow(fR, 2) - math.Pow(fi, 2))
		for fa := 0.0; fa <= 2*PI; fa = fa + fA/fr {
			rayX := rt.Ray{Point: transmitter.Point, Direction: []float64{fr * math.Cos(fa), fr * math.Sin(fa), fi}}
			wg.Add(1)
			go func(rayX rt.Ray, count_rays int) {
				Raytrace(&rayid, rayX, 1, 0, obstacles, 0, ch)
				wg.Done()
			}(rayX, count_rays)
			count_rays++
		}
	}

	wg.Wait()

	elapsed := time.Since(start)
	fmt.Println("\nDone!!!\nProcessed ", count_rays, " rays in ", elapsed)
	rt.Data.Process.TimeTaken = elapsed
	rt.Data.Process.NumCores = 2

	/*Print the sotred data into the out.json file*/
	pinbytes, _ := json.MarshalIndent(rt.Data, "", "\t")
	jd := json.NewEncoder(fid)
	json.MarshalIndent(jd, "", "\t")
	fmt.Fprintln(fid, string(pinbytes))
	fid.Close()
}
