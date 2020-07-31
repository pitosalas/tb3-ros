//go:generate statik -src=./templates -f
package main

import (
	"fmt"
	"io/ioutil"
	"os"
	"strings"
	"text/template"

	_ "github.com/pitosalas/tb3-ros/server/cmd/serverctl/statik"
	"github.com/pitosalas/tb3-ros/server/internal/server"

	"github.com/rakyll/statik/fs"
	"github.com/urfave/cli/v2"
)

func GenerateCommand() *cli.Command {
	return &cli.Command{
		Name:  "generate",
		Usage: "generate a new tb3-ros server",
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
		Action: generateAction(),
	}
}

func generateAction() cli.ActionFunc {
	return func(ctx *cli.Context) error {
		path := ctx.String("config")

		services, err := server.ParseConfig(path)
		if err != nil {
			return fmt.Errorf("failed to parse config: %v", err)
		}

		t, err := compileTemplates("/")
		if err != nil {
			return fmt.Errorf("failed to read templates: %v", err)
		}

		f, err := os.OpenFile(composeFileName, os.O_RDWR|os.O_CREATE|os.O_TRUNC, 0644)
		if err != nil {
			return fmt.Errorf("failed to create compose file: %v", err)
		}

		if err := t.ExecuteTemplate(f, "docker-compose", services); err != nil {
			return fmt.Errorf("failed to write to template: %v", err)
		}

		return nil
	}
}

func compileTemplates(dir string) (*template.Template, error) {
	tpl := template.New("")

	box, err := fs.New()
	if err != nil {
		return nil, fmt.Errorf("failed to find template directory: %v", err)
	}

	err = fs.Walk(box, dir, func(path string, info os.FileInfo, _ error) error {
		// Skip non-templates.
		if info.IsDir() || !strings.HasSuffix(path, ".tmpl") {
			return nil
		}

		f, err := box.Open(path)
		if err != nil {
			return err
		}
		sl, err := ioutil.ReadAll(f)
		if err != nil {
			return err
		}

		tpl.Parse(string(sl))

		return nil
	})

	return tpl, err
}
