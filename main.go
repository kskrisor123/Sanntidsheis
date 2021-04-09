package main

import (
	"fmt"
	"time"

	"../Sanntidsheis/FSM"
)

func main() {
	fmt.Println("test", time.Second)
	FSM.InternalControl()

}
