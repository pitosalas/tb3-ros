package main

import (
	"fmt"
	"log"
	"os/exec"

	"github.com/urfave/cli/v2"
)

func DownCommand() *cli.Command {
	return &cli.Command{
		Name:  "up",
		Usage: "start a tb3-ros server",
		Flags: []cli.Flag{
			&cli.StringFlag{
				Name:        "config",
				Usage:       "Config file for defining a tb3-ros server",
				Aliases:     []string{"c"},
				DefaultText: "config.yaml",
				TakesFile:   true,
				Required:    true,
			},
		},
		Action: downAction(),
	}
}

func downAction() cli.ActionFunc {
	return func(ctx *cli.Context) error {
		err := generateAction()(ctx)
		if err != nil {
			return fmt.Errorf("failed to generate compose file: %v", err)
		}

		args := []string{
			"-f",
			composeFileName,
			"down",
		}

		cmd := exec.Command("docker-compose", args...)

		log.Println("stopping server...")
		if out, err := cmd.CombinedOutput(); err != nil {
			return fmt.Errorf("failed to stop docker-compose: %s", out)
		}

		return cmd.Wait()
	}
}
