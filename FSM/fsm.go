package FSM

import (
	"fmt"
	"time"

	"project.com/driver/elevio"
)

/*
func goToFloor(floorRequest int, elevatorState <-chan int) {
    currentFloor := <- elevatorState

    fmt.Println(currentFloor)

}
*/

const numFloors = 4
const numButtons = 3

type ElevatorState int

const (
	IDLE      = 0
	RUNNING   = 1
	DOOR_OPEN = 2
)

type Direction int

const (
	UP    = 1
	DOWN  = -1
	STILL = 0
)

type Elev struct {
	State ElevatorState
	Dir   Direction
	Floor int
	Queue [numFloors][numButtons]bool
}

type Keypress struct {
	Floor              int
	Btn                elevio.ButtonType
	DesignatedElevator int
	Done               bool
}

func ordersAbove(elevator Elev) bool {
	currentFloor := elevator.Floor
	for i := currentFloor + 1; i < numFloors; i++ {
		if elevator.Queue[i][0] || elevator.Queue[i][1] || elevator.Queue[i][2] {
			return true
		}
	}
	return false
}

func ordersBelow(elevator Elev) bool {
	currentFloor := elevator.Floor
	for i := currentFloor - 1; i > -1; i-- {
		if elevator.Queue[i][0] || elevator.Queue[i][1] || elevator.Queue[i][2] {
			return true
		}
	}
	return false
}

func ordersInFloor(elevator Elev) bool {
	for i := 0; i < 3; i++ {
		if elevator.Queue[elevator.Floor][i] {
			return true
		}
	}
	return false
}

func DeleteOrder(elevator *Elev) {
	deletedFloor := elevator.Floor
	for i := 0; i < numButtons; i++ {
		elevator.Queue[elevator.Floor][i] = false
	}
	fmt.Println("Order deleted at ", deletedFloor)
}

func DeleteAllOrders(elevator *Elev) {
	for btn := 0; btn < numButtons; btn++ {
		for floor := 0; floor < numFloors; floor++ {
			elevator.Queue[floor][btn] = false
			fmt.Println(elevator.Queue[floor][btn])
		}
	}

}

func chooseElevatorDir(elevator Elev) elevio.MotorDirection {
	switch elevator.Dir {
	case STILL:
		if ordersAbove(elevator) {
			return elevio.MD_Up
		} else if ordersBelow(elevator) {
			return elevio.MD_Down
		} else {
			return elevio.MD_Stop
		}
	case UP:
		if ordersAbove(elevator) {
			return elevio.MD_Up
		} else if ordersBelow(elevator) {
			return elevio.MD_Down
		} else {
			return elevio.MD_Stop
		}

	case DOWN:
		if ordersBelow(elevator) {
			return elevio.MD_Down
		} else if ordersAbove(elevator) {
			return elevio.MD_Up
		} else {
			return elevio.MD_Stop
		}
	}
	return elevio.MD_Stop
}

func shouldStop(elevator Elev) bool {
	switch elevator.Dir {
	case UP:
		return elevator.Queue[elevator.Floor][elevio.BT_HallUp] ||
			elevator.Queue[elevator.Floor][elevio.BT_Cab] ||
			!ordersAbove(elevator)
	case DOWN:
		return elevator.Queue[elevator.Floor][elevio.BT_HallDown] ||
			elevator.Queue[elevator.Floor][elevio.BT_Cab] ||
			!ordersBelow(elevator)
	case STILL:
	}
	return false
}

func motorDirToElevDir(direction elevio.MotorDirection) Direction {
	if direction == elevio.MD_Up {
		return UP
	} else if direction == elevio.MD_Down {
		return DOWN
	} else {
		return STILL
	}
}

func printQueue(elevator Elev) {
	for button := 0; button < numButtons; button++ {
		for floor := 0; floor < numFloors; floor++ {
			fmt.Println(elevator.Queue[floor][button])
		}
	}
}

func FsmInit() {

	elevator.State = IDLE
	// Needs to start in a well-defined state
	for elevator.Floor = elevio.GetFloor(); elevator.Floor < 0; elevator.Floor = elevio.GetFloor() {
		elevio.SetMotorDirection(elevio.MD_Up)
		time.Sleep(1 * time.Second)
	}
	elevio.SetMotorDirection(elevio.MD_Stop)
	fmt.Println("FSM initialized!")
}

func FsmUpdateFloor(newFloor int) {
	elevator.Floor = newFloor
}

func removeButtonLamps(elevator Elev) {
	elevio.SetButtonLamp(elevio.BT_Cab, elevator.Floor, false)
	elevio.SetButtonLamp(elevio.BT_HallDown, elevator.Floor, false)
	elevio.SetButtonLamp(elevio.BT_HallUp, elevator.Floor, false)
}

var dir elevio.MotorDirection

func fsm(doorsOpen chan<- int) {
	for {
		switch elevator.State {
		case IDLE:
			if ordersAbove(elevator) {
				//println("order above,going up, current Floor: ", Floor)
				dir = elevio.MD_Up
				elevator.Dir = motorDirToElevDir(dir)
				elevio.SetMotorDirection(dir)
				elevator.State = RUNNING
			}
			if ordersBelow(elevator) {
				//println("order below, going down, current Floor: ", Floor)
				dir = elevio.MD_Down
				elevator.Dir = motorDirToElevDir(dir)
				elevio.SetMotorDirection(dir)
				elevator.State = RUNNING
			}
			if ordersInFloor(elevator) {
				//println("order below, going down, current Floor: ", Floor)
				elevator.State = DOOR_OPEN
			}
		case RUNNING:
			if ordersInFloor(elevator) { // this is the problem : the floor is being kept constant at e.g. 2 while its moving
				dir = elevio.MD_Stop
				elevator.Dir = motorDirToElevDir(dir)
				elevio.SetMotorDirection(dir)
				elevator.State = DOOR_OPEN
			}
		case DOOR_OPEN:
			fmt.Println("printing queue")
			printQueue(elevator)
			elevio.SetDoorOpenLamp(true)
			dir = elevio.MD_Stop
			elevio.SetMotorDirection(dir)
			println("DOOR OPEN")
			DeleteOrder(&elevator)
			elevator.State = IDLE
			doorsOpen <- elevator.Floor
			timer1 := time.NewTimer(2 * time.Second)
			<-timer1.C
			elevio.SetDoorOpenLamp(false)
			removeButtonLamps(elevator)
			println("DOOR CLOSE")

		}
	}

}

var elevator = Elev{
	State: IDLE,
	Dir:   STILL,
	Floor: 0,
	Queue: [numFloors][numButtons]bool{},
}

// InternalControl .. Responsible for internal control of a single elevator
func InternalControl() {
	println("Connecting to server")
	elevio.Init("localhost:15657", numFloors)

	FsmInit()
	drvButtons := make(chan elevio.ButtonEvent)
	drvFloors := make(chan int)
	drvStop := make(chan bool)
	doorsOpen := make(chan int)
	CompletedOrder := make(chan elevio.ButtonEvent, 100)

	//drv_obstr := make(chan bool)
	//go elevio.PollObstructionSwitch(drv_obstr)

	go elevio.PollButtons(drvButtons)
	go elevio.PollFloorSensor(drvFloors)
	go elevio.PollStopButton(drvStop)
	go fsm(doorsOpen)
	for {
		select {
		case floor := <-drvFloors: //Sensor senses a new floor
			//println("updating floor:", floor)
			FsmUpdateFloor(floor)
		case drvOrder := <-drvButtons: // a new button is pressed on this elevator
			//ch.DelegateOrder <- drvOrder //Delegate this order
			fmt.Println("New order")
			fmt.Println(drvOrder)
			elevator.Queue[drvOrder.Floor][int(drvOrder.Button)] = true
			elevio.SetButtonLamp(drvOrder.Button, drvOrder.Floor, true)
		/*case ExtOrder := <-ch.TakeExternalOrder:
		AddOrder(ExtOrder)*/
		case floor := <-doorsOpen:
			order_OutsideUp_Completed := elevio.ButtonEvent{
				Floor:  floor,
				Button: elevio.BT_HallUp,
			}
			order_OutsideDown_Completed := elevio.ButtonEvent{
				Floor:  floor,
				Button: elevio.BT_HallDown,
			}
			order_Inside_Completed := elevio.ButtonEvent{
				Floor:  floor,
				Button: elevio.BT_Cab,
			}
			CompletedOrder <- order_OutsideUp_Completed
			CompletedOrder <- order_OutsideDown_Completed
			CompletedOrder <- order_Inside_Completed

		case <-drvStop:
			elevio.SetMotorDirection(elevio.MD_Stop)
			time.Sleep(3 * time.Second)

		}

	}
}
