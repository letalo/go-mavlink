package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	// "github.com/SpaceLeap/go-mavlink/mavlink/minimal"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
	"github.com/ungerik/go-dry"
	"github.com/ungerik/goserial"
)

var (
	port       string
	maxPackets int
	quitAfter  time.Duration

	stop bool
)

func main() {
	flag.StringVar(&port, "port", "", "Serial port to connect to")
	flag.IntVar(&maxPackets, "max", 10, "Quit program after this number of packets")
	flag.DurationVar(&quitAfter, "quitafter", time.Second*3, "Quit program after this duration")
	flag.Parse()

	if port == "" && flag.NArg() == 0 {
		fmt.Fprintln(os.Stderr, "Call mavlink -port=PORT")
		flag.PrintDefaults()
		fmt.Fprintln(os.Stderr, "\nAvailable as PORT are:")
		for _, p := range goserial.ListPorts() {
			fmt.Fprintln(os.Stderr, "  ", p)
		}
		return
	}
	if port == "" {
		port = flag.Arg(0)
	}

	serialConn, err := goserial.OpenPort(&goserial.Config{Name: port, Baud: 115200, Timeout: time.Second})
	if err != nil {
		log.Fatal(err)
	}
	log.Println("Opened serial port", port)
	defer func() {
		log.Println("Closed serial port", port, "with error", serialConn.Close())
	}()

	//conn := mavlink.NewConnection(serialConn, 99)

	err = mavlink.Send(serialConn, 0, 0, 0, common.NewHeartbeat())
	if err != nil {
		log.Println(err)
	}

	err = mavlink.Send(serialConn, 0, 0, 0, &common.Ping{})
	if err != nil {
		log.Println(err)
	}

	err = mavlink.Send(serialConn, 0, 0, 0, &common.ParamRequestList{})
	if err != nil {
		log.Println(err)
	}

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
