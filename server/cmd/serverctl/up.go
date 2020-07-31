package main

import (
	"fmt"
	"log"
	"os"
	"os/exec"

	"github.com/pitosalas/tb3-ros/server/internal/server"
	"github.com/urfave/cli/v2"
)

func UpCommand() *cli.Command {
	return &cli.Command{
		Name:  "up",
		Usage: "start the tb3-ros server",
		Flags: []cli.Flag{
			&cli.StringFlag{
				Name:        "config",
				Usage:       "Config file for defining a tb3-ros server",
				Aliases:     []string{"c"},
				DefaultText: "config.yaml",
				TakesFile:   true,
				Required:    true,
			},
			&cli.BoolFlag{
				Name:    "detach",
				Usage:   "Detached mode: Run containers in the background, print new container names.",
				Aliases: []string{"d"},
				Value:   false,
			},
		},
		Action: upAction(),
	}
}

func upAction() cli.ActionFunc {
	return func(ctx *cli.Context) error {
		err := generateAction()(ctx)
		if err != nil {
			return fmt.Errorf("failed to generate compose file: %v", err)
		}

		// Create data directories.
		config, _ := server.ParseConfig(ctx.String("config"))
		for _, d := range config.DesktopList {
			path := d.Volume.Path
			if _, err := os.Stat(path); err != nil {
				if os.IsNotExist(err) {
					log.Printf("%s not found, creating directory...", path)
					os.MkdirAll(path, 0700)
				} else {
					log.Fatalf("failed to lookup data directory: %v", err)
				}
			}
		}

		// Start compose file.
		args := []string{
			"-f",
			composeFileName,
			"up",
		}
		if ctx.Bool("detach") {
			args = append(args, "-d")
		}

		cmd := exec.Command("docker-compose", args...)

		log.Println("starting server...")
		if out, err := cmd.CombinedOutput(); err != nil {
			return fmt.Errorf("failed to start docker-compose: %s", out)
		}

		return cmd.Wait()
	}
}
