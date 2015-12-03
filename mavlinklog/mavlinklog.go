package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/letalo/go-mavlink/mavlink"
	// "github.com/letalo/go-mavlink/mavlink/common"
	"github.com/letalo/go-mavlink/mavlink/ardupilotmega"
	tarm "github.com/tarm/goserial"
)

var (
	port       string
	baud       int
	maxPackets int
	quitAfter  time.Duration
	timeout    time.Duration
	debug      bool

	stop bool
)

func main() {
	// common.Init()
	ardupilotmega.Init()

	flag.StringVar(&port, "port", "", "Serial port to connect to")
	flag.IntVar(&baud, "baud", 115200, "Speed of the connection")
	flag.IntVar(&maxPackets, "max", 10, "Quit program after this number of packets")
	flag.DurationVar(&quitAfter, "quitafter", time.Second*3, "Quit program after this duration")
	flag.DurationVar(&timeout, "timeout", time.Second, "Read timeout per packet")
	flag.BoolVar(&debug, "debug", false, "Log everything for debugging")
	flag.Parse()

	if debug {
		mavlink.LogEverything(log.New(os.Stderr, "", log.LstdFlags))
	}

	if port == "" {
		if flag.NArg() > 0 {
			port = flag.Arg(0)
		} else {
			ports := serial.ListPorts()
			if len(ports) == 1 {
				port = ports[0]
			} else {
				fmt.Fprintln(os.Stderr, "Call with -port=PORT")
				flag.PrintDefaults()
				fmt.Fprintln(os.Stderr, "\nAvailable as PORT are:")
				for _, p := range ports {
					fmt.Fprintln(os.Stderr, "  ", p)
				}
				return
			}
		}
	}

	// serialConn, err := serial.OpenDefault(port, serial.Baud(baud), timeout)
	serialConn, err := tarm.OpenPort(&tarm.Config{Name: port, Baud: baud})
	if err != nil {
		log.Fatal(err)
	}
	log.Println("Opened serial port", port)
	defer func() {
		log.Println("Closed serial port", port, "with error", serialConn.Close())
	}()

	//conn := mavlink.NewConnection(serialConn, 99)

	// err = mavlink.Send(serialConn, 0, 0, 0, common.NewHeartbeat())
	// if err != nil {
	// 	log.Println(err)
	// }

	// err = mavlink.Send(serialConn, 0, 0, 0, &common.Ping{})
	// if err != nil {
	// 	log.Println(err)
	// }

	// err = mavlink.Send(serialConn, 0, 0, 0, &common.ParamRequestList{})
	// if err != nil {
	// 	log.Println(err)
	// }

	go func() {
		dry.WaitForStdin("Press any key to quit")
		stop = true
	}()

	time.AfterFunc(quitAfter, func() { stop = true })

	for i := 0; i < maxPackets && !stop; i++ {
		packet, err := mavlink.Receive(serialConn)
		if err == nil {
			log.Println(packet)
		} else {
			log.Println(err)
		}
	}
}
