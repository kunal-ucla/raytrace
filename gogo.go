package main

import (
	"bytes"
	"encoding/json"
	//"encoding/json"
	"os"

	"fmt"
)

type Person struct {
	Name  string
	Habit string
	In    int `json:"-"`
}

func main() {
	if 1 == 2 {
		fmt.Println("lol")
	} else if 3 == 2 {
		fmt.Println("pol")
	}
	for i := 0; i < 3; i++ {
		fmt.Println(i)
	}
	for i := 0; i < 3; i++ {
		fmt.Println("HI")
	}
	fid, ferr := os.Create("test.csv")
	defer fid.Close()
	if ferr != nil {
		fmt.Println("Unable to open the file Erro is = ", ferr)
		return
	}

	var p Person
	p.Habit = "popop"
	p.Name = "kjklj"
	p.In = 999
	i := 3

	pinbytes, jerr := json.Marshal(p)
	var dbytes []byte
	buf := bytes.NewBuffer(dbytes)
	json.Indent(buf, pinbytes, "@", "-")
	if jerr != nil {
		fmt.Print("Found erro r", jerr)
		return
	}
	// for i := 0; i < 10; i++ {
	_ = i
	fmt.Fprintf(fid, "%s", buf.Bytes())

	// }

	fid.Close()
}
