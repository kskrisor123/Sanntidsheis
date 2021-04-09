package main

import (
	"fmt"
	"time"

	"../driver/elevio"
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
	for i := currentFloor + 1; i < numFloors-1; i++ {
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

var elevator = Elev{
	State: IDLE,
	Dir:   STILL,
	Floor: 0,
	Queue: [numFloors][numButtons]bool{},
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
			println("DOOR CLOSE")

		}
	}

}

// InternalControl .. Responsible for internal control of a single elevator
func main() {
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
		}

	}
}

/*
type StateMachineChannels struct {
	OrderComplete  chan int
	Elevator       chan Elev
	StateError     chan error
	NewOrder       chan Keypress
	ArrivedAtFloor chan int
}

// RunElevator called as a goroutine; runs elevator and updates governor for changes
func RunElevator(ch StateMachineChannels) {
	elevator := Elev{
		State: Idle,
		Dir:   DirStop,
		Floor: hw.GetFloorSensorSignal(),
		Queue: [NumFloors][NumButtons]bool{},
	}
	doorTimedOut := time.NewTimer(3 * time.Second)
	engineErrorTimer := time.NewTimer(3 * time.Second)
	doorTimedOut.Stop()
	engineErrorTimer.Stop()
	orderCleared := false
	ch.Elevator <- elevator

	for {
		select {
		case newOrder := <-ch.NewOrder:
			if newOrder.Done {
				elevator.Queue[newOrder.Floor][BtnUp] = false
				elevator.Queue[newOrder.Floor][BtnDown] = false
				orderCleared = true
			} else {
				elevator.Queue[newOrder.Floor][newOrder.Btn] = true
			}

			switch elevator.State {
			case Idle:
				elevator.Dir = chooseDirection(elevator)
				hw.SetMotorDirection(elevator.Dir)
				if elevator.Dir == DirStop {
					elevator.State = DoorOpen
					hw.SetDoorOpenLamp(1)
					doorTimedOut.Reset(3 * time.Second)
					go func() { ch.OrderComplete <- newOrder.Floor }()
					elevator.Queue[elevator.Floor] = [NumButtons]bool{}
				} else {
					elevator.State = Moving
					engineErrorTimer.Reset(3 * time.Second)
				}

			case Moving:
			case DoorOpen:
				if elevator.Floor == newOrder.Floor {
					doorTimedOut.Reset(3 * time.Second)
					go func() { ch.OrderComplete <- newOrder.Floor }()
					elevator.Queue[elevator.Floor] = [NumButtons]bool{}
				}

			case Undefined:
			default:
				fmt.Println("Fatal error: Reboot system")
			}
			ch.Elevator <- elevator

		case elevator.Floor = <-ch.ArrivedAtFloor:
			fmt.Println("Arrived at floor", elevator.Floor+1)
			if shouldStop(elevator) ||
				(!shouldStop(elevator) && elevator.Queue == [NumFloors][NumButtons]bool{} && orderCleared) {
				orderCleared = false
				hw.SetDoorOpenLamp(1)
				engineErrorTimer.Stop()
				elevator.State = DoorOpen
				hw.SetMotorDirection(DirStop)
				doorTimedOut.Reset(3 * time.Second)
				elevator.Queue[elevator.Floor] = [NumButtons]bool{}
				go func() { ch.OrderComplete <- elevator.Floor }()

			} else if elevator.State == Moving {
				engineErrorTimer.Reset(3 * time.Second)
			}
			ch.Elevator <- elevator

		case <-doorTimedOut.C:
			hw.SetDoorOpenLamp(0)
			elevator.Dir = chooseDirection(elevator)
			if elevator.Dir == DirStop {
				elevator.State = Idle
				engineErrorTimer.Stop()
			} else {
				elevator.State = Moving
				engineErrorTimer.Reset(3 * time.Second)
				hw.SetMotorDirection(elevator.Dir)
			}
			ch.Elevator <- elevator

		case <-engineErrorTimer.C:
			hw.SetMotorDirection(DirStop)
			elevator.State = Undefined
			fmt.Println("\x1b[1;1;33m", "Engine Error - Go offline", "\x1b[0m")
			for i := 0; i < 10; i++ {
				if i%2 == 0 {
					hw.SetStopLamp(1)
				} else {
					hw.SetStopLamp(0)
				}
				time.Sleep(time.Millisecond * 200)
			}
			hw.SetMotorDirection(elevator.Dir)
			ch.Elevator <- elevator
			engineErrorTimer.Reset(5 * time.Second)
		}
	}
}*/

/*
Dere må dekalrer een matrise [floor][buttons]boool
floor (må finne ut hvileke etajse heisen er i)
    hvis ikk i etasje send til nærnmeste etajse
state_Elevator = idle



deklarer button timer
dekalrer motor kræsj timer

select case
    case tar inn channel order
        lagre den i matrisen

        state machine
            case heise er i idle
                motor = kalkuler_motor__retning(floor, ordre)
                state_Elevator = running
                starte motor_timer
            case heis er i running

            case


    casse channel finis door timer
        slår vi av door lyse
        sjekker og eventuelt kalkulerer nå moor retning
            hvis det er andre order vi har så starter motor og starte mtoor timer
            og setter state til running

        hvis det ikke er sant
            så setter state til idle

    case channel motortimer går ut
        så må vi slå alarm

    case channel floor
        if true er heisen i etajsen den vurder stoppe()
            starte door timer
            slå på door lys
            stoppe motor timer

    case hei er i door










*/
