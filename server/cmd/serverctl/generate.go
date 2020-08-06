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

type generateFunc func(*template.Template, *server.Services) (string, error)

// GenerateCommand is a CLI command for generating a new tb3-ros server.
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

		if err := mkBuildDir(services.Config.Server.BuildPath); err != nil {
			return fmt.Errorf("failed to write to build directory: %v", err)
		}

		for _, gFunc := range []generateFunc{
			generateComposeFile,
			generateOverviewFile,
		} {
			if f, err := gFunc(t, services); err != nil {
				return fmt.Errorf("failed to write to %s: %v", f, err)
			}
		}

		return nil
	}
}

func mkBuildDir(directory string) error {
	if err := os.RemoveAll(directory); err != nil {
		return fmt.Errorf("failed to remove old build directory: %v", err)
	}

	if err := os.MkdirAll(directory, 0700); err != nil {
		return fmt.Errorf("failed to recreate build directory: %v", err)
	}

	return nil
}

func generateComposeFile(t *template.Template, s *server.Services) (string, error) {
	fPath := buildPath(s.Config.Server.BuildPath, server.ComposeFileName)

	f, err := os.OpenFile(fPath, os.O_RDWR|os.O_CREATE|os.O_TRUNC, os.ModePerm)
	if err != nil {
		return fPath, fmt.Errorf("failed to create compose file: %v", err)
	}

	if err := t.ExecuteTemplate(f, "docker-compose", s); err != nil {
		return fPath, fmt.Errorf("failed to write to template: %v", err)
	}

	return fPath, nil
}

func generateOverviewFile(t *template.Template, s *server.Services) (string, error) {
	fPath := buildPath(s.Config.Server.BuildPath, server.OverviewFileName)

	f, err := os.OpenFile(fPath, os.O_RDWR|os.O_CREATE|os.O_TRUNC, os.ModePerm)
	if err != nil {
		return fPath, fmt.Errorf("failed to create overview file: %v", err)
	}

	if err := t.ExecuteTemplate(f, "overview", s); err != nil {
		return fPath, fmt.Errorf("failed to write to template: %v", err)
	}

	return fPath, nil
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
