# VDA5050 Schema Registry

This directory stores the local schema registry used by the VDA5050 mission
package validator.

The runtime validator currently supports configured VDA5050 major-version-3
messages (`3.0.0`, `3.0.1`, `3.1.0` by default) and applies semantic checks in
code. Replace the schema anchors with exact upstream schemas when adopting a new
minor/patch release.
