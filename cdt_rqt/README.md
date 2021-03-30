# cdt_rqt

Graphical interface for the CDT challenge

## Dependencies

In order to generate the pyqt resources, you need `pyrcc5`:

```sh
sudo apt install pyqt5-dev-tools
```

## Generating resources

To regenerate/update the resources, go to the `resource` folder and run:

```sh
pyrcc5 resources.qrc -o ../src/cdt_rqt/resources.py
```