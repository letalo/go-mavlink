package main

import (
	"flag"
	"fmt"
	"log"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/ungerik/go-dry"
	"github.com/ungerik/goserial"
)

var (
	port string
	stop bool
)

func main() {
	flag.StringVar(&port, "port", "", "Serial port to connect to")
	flag.Parse()

	if port == "" && flag.NArg() == 0 {
		fmt.Println("Call mavlink -port=PORT")
		flag.PrintDefaults()
		fmt.Println("\nAvailable as PORT are:")
		for _, p := range goserial.ListPorts() {
			fmt.Println("  ", p)
		}
		return
	}
	if port == "" {
		port = flag.Arg(0)
	}

	serialConn, err := goserial.OpenPort(&goserial.Config{Name: port, Baud: 115200})
	if err != nil {
		log.Fatal(err)
	}
	log.Println("Opened serial port", port)
	defer log.Println("Closed serial port", port, "with error", serialConn.Close())

	conn := mavlink.NewConnection(serialConn, 99)

	go func() {
		dry.WaitForStdin("Press any key to quit")
		stop = true
	}()

	for !stop {
		packet, err := conn.Receive()
		if err == nil {
			log.Println(packet)
		} else {
			log.Println(err)
		}
	}
}
