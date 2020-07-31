package main

import "path/filepath"

func buildPath(buildDir string, file string) string {
	return filepath.Join(buildDir, file)
}
