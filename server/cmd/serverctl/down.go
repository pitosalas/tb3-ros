package main

import (
	"fmt"
	"log"
	"os/exec"

	"github.com/pitosalas/tb3-ros/server/internal/server"
	"github.com/urfave/cli/v2"
)

func DownCommand() *cli.Command {
	return &cli.Command{
		Name:  "down",
		Usage: "stop the tb3-ros server",
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

		services, _ := server.ParseConfig(ctx.String("config"))

		args := []string{
			"-f",
			buildPath(services.Config.Server.BuildPath, server.ComposeFileName),
			"down",
		}

		cmd := exec.Command("docker-compose", args...)

		log.Println("stopping server...")
		out, err := cmd.CombinedOutput()
		if err != nil {
			return fmt.Errorf("failed to stop server: %s", err)
		}
		log.Printf("%s", out)

		return nil
	}
}
