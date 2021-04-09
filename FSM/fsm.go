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

func ordersAbove(elevator Elev) bool {
	for floor := elevator.Floor + 1; floor < numFloors; floor++ {
		for btn := 0; btn < numButtons; btn++ {
			if elevator.Queue[floor][btn] {
				return true
			}
		}
	}
	return false
}

func ordersBelow(elevator Elev) bool {
	for floor := 0; floor < elevator.Floor; floor++ {
		for btn := 0; btn < numButtons; btn++ {
			if elevator.Queue[floor][btn] {
				return true
			}
		}
	}
	return false
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

func main() {

	//numFloors := 4

	//queue := make([]elevio.ButtonEvent, 0)

	elevio.Init("localhost:15657", numFloors)

	//var d elevio.MotorDirection = elevio.MD_Up
	//elevio.SetMotorDirection(d)

	drv_buttons := make(chan elevio.ButtonEvent)
	drv_floors := make(chan int)
	drv_obstr := make(chan bool)
	drv_stop := make(chan bool)

	go elevio.PollButtons(drv_buttons)
	go elevio.PollFloorSensor(drv_floors)
	go elevio.PollObstructionSwitch(drv_obstr)
	go elevio.PollStopButton(drv_stop)

	elevator := Elev{
		State: IDLE,
		Dir:   STILL,
		Floor: elevio.GetFloor(),
		Queue: [numFloors][numButtons]bool{},
	}

	doorTimer := time.NewTimer(3 * time.Second)
	engineErrorTimer := time.NewTimer(3 * time.Second)
	doorTimer.Stop()
	engineErrorTimer.Stop()
	orderCleared := false

	for {
		select {
		case a := <-drv_buttons:
			fmt.Println("New order", a.Floor, a.Button)
			elevio.SetButtonLamp(a.Button, a.Floor, true)
			elevator.Queue[int(a.Button)][a.Floor] = true

			switch elevator.State {
			case IDLE:
				fmt.Println("IDLE")
				direction := chooseElevatorDir(elevator)
				elevator.Dir = motorDirToElevDir(direction)
				elevio.SetMotorDirection(direction)
				if elevator.Dir == STILL {
					elevator.State = DOOR_OPEN
					elevio.SetDoorOpenLamp(true)
					doorTimer.Reset(3 * time.Second)
					//go func() { ch.OrderComplete <- newOrder.Floor }()
					elevator.Queue[elevator.Floor] = [numButtons]bool{}
				} else {
					elevator.State = RUNNING
					engineErrorTimer.Reset(3 * time.Second)
				}

			case RUNNING:
				fmt.Println("Running")
			case DOOR_OPEN:
				if elevator.Floor == a.Floor {
					fmt.Println("Door open")
					doorTimer.Reset(3 * time.Second)
					//go func() { ch.OrderComplete <- newOrder.Floor }()
					elevator.Queue[elevator.Floor] = [numButtons]bool{}
					elevio.SetButtonLamp(a.Button, a.Floor, false)
				}
			}

		case a := <-drv_floors:
			fmt.Println("Arrived at floor", elevator.Floor+1)
			elevator.Floor = a
			if shouldStop(elevator) || (!shouldStop(elevator) && elevator.Queue == [numFloors][numButtons]bool{} && orderCleared) {
				orderCleared = false
				elevio.SetDoorOpenLamp(true)
				engineErrorTimer.Stop()
				elevator.State = DOOR_OPEN
				elevio.SetMotorDirection(elevio.MD_Stop)
				doorTimer.Reset(3 * time.Second)
				elevator.Queue[elevator.Floor] = [numButtons]bool{}
				//go func() { ch.OrderComplete <- elevator.Floor }()

			} else if elevator.State == RUNNING {
				engineErrorTimer.Reset(3 * time.Second)
			}
			//ch.Elevator <- elevator

		case <-doorTimer.C:
			fmt.Println("Timer out")
			elevio.SetDoorOpenLamp(false)
			direction := chooseElevatorDir(elevator)
			elevator.Dir = motorDirToElevDir(direction)
			if elevator.Dir == STILL {
				elevator.State = IDLE
				engineErrorTimer.Stop()
			} else {
				elevator.State = RUNNING
				engineErrorTimer.Reset(3 * time.Second)

				elevio.SetMotorDirection(direction)
			}
			//ch.Elevator <- elevator

		case a := <-drv_obstr:
			fmt.Printf("%+v\n", a)
			/*if a {
				elevio.SetMotorDirection(elevio.MD_Stop)
			} else {
				elevio.SetMotorDirection(d)
			}*/

		case a := <-drv_stop:
			fmt.Printf("%+v\n", a)
			/*for f := 0; f < numFloors; f++ {
				for b := elevio.ButtonType(0); b < 3; b++ {
					elevio.SetButtonLamp(b, f, false)
				}
			}*/
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
