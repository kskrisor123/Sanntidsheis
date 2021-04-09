package ordermanager

import (
	"project.com/driver/elevio"
)

type orderMess struct {
	Floor     int
	Direction int
	Timelist  []int
}

//Wait for new hall orders
func ordermanager() {

	drv_buttons := make(chan elevio.ButtonEvent)
	orderMessage := make(chan orderMess)
	var hallbtn elevio.ButtonEvent
	var order orderMess

	for {
		select {
		case hallbtn = <-drv_buttons:
			if hallbtn.Button == 2 {
			} else {

			}
		case order = <-orderMessage:
		}
	}
}

//Start groutine to handle order

//Do cost calculation (send to cost function)

//If itself: send to fsm and broadcast
//Else: send to correct elevator, and monitor.

//Wait for order broadcast

//Start goroutine to deal with broadcast

//If taking order: send to fsm (wait 1 sec) and broadcast order accept
//Else: Wait for order accept, then wait for order finish
