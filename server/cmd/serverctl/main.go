package main

import (
	"log"
	"os"

	"github.com/urfave/cli/v2"
)

var (
	BuildVersion = "development"
)

func main() {
	app := &cli.App{
		Version: BuildVersion,
		Commands: []*cli.Command{
			GenerateCommand(),
			UpCommand(),
			DownCommand(),
		},
	}

	err := app.Run(os.Args)
	if err != nil {
		log.Fatal(err)
	}
}
