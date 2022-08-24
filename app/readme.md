# How to create a container App for PLCnext

This template implements everything you will need to creating a container App for PLCnext Technology.

To get started, this templates includes a very small hello-world web-server example.

## Get started (ARM)

Simply [build the app](https://store.plcnext.help/st/PLCnext_App_Integration_Guide/PLCnext_Apps/Building_a_PLCnext_App.htm) and [install](https://www.plcnext.help/te/WBM/Administration_PLCnext_Apps.htm) it via the Web Based Management of your controller.

## Get started (x86)

1. Open the `initscript.sh`, find the `APP configuration` section and change the archive name from `hello-world.tar.gz` to `hello-world_x86.tar.gz`

2. Then simply [build the app](https://store.plcnext.help/st/PLCnext_App_Integration_Guide/PLCnext_Apps/Building_a_PLCnext_App.htm) and [install](https://www.plcnext.help/te/WBM/Administration_PLCnext_Apps.htm) it via the Web Based Management of your controller.

## Basic configuration

Minimal configuration you have to do to adapt the template for your own app.

### app_info.json

Following parameter have to be changed:

- `name` of your App
- `identifier` - can be obtained after creating a new app in the PLCnext store
- `target` - need to be compatible to your container architecture and can be a list of PLCnext targets like "AXC F 2152,AXC F 1152"
- `minfirmware_version` - need to be at least 22.6.0 for this type of app
- `manufacturer`

Follow this [link](https://store.plcnext.help/st/PLCnext_App_Integration_Guide/Apps_parts/Metadata.htm#bookmark) for more information to the metadata.

### initscript.sh

Find the `APP configuration` section and edit the two available paramter.

Specify your **image(s)** and their accociated names by adding appropriate lines.

```bash
IMAGES[image_name]=image_archive
```

- `image_name` = the image name in the container system like hello-world
- `image_archive` = the name of your saved image archive e.g. hello-world_x86.tar.gz

The archive(s) need to be located in the images folder.

### app-compose.yml

Write your own app-compose.yml.

For information about User Configuration, Volumes and environment variables, please look at the **Advanced configuration** chapter.

After your basic configuration is done, you can [build the app](https://store.plcnext.help/st/PLCnext_App_Integration_Guide/PLCnext_Apps/Building_a_PLCnext_App.htm) and [install](https://www.plcnext.help/te/WBM/Administration_PLCnext_Apps.htm) your app.

## Advanced configuration

### User Configuration

Compose offers the option to load variables from a file. In PLCnext we will use this to allow user configuration without beeing able to edit the whole compose.yml. This can be used for example port mappings, user, persistent storage pathes or environment variables passed into the container.

As an example the external port mapping in this template is available as a user editable variable.

First of all you have to use the variable in your compose.yml. Our recommendation is to set a default if VARIABLE is unset or empty in the environment with the following pattern `${VARIABLE:-default}`. For more information follow the [link](https://docs.docker.com/compose/environment-variables/) into the official docker documentation.

If now a key value pair with the VARIABLE name as key is set in the `user.env`. It will be used upon restart. The environment file which will be used, is editable for the admin user under `/opt/plcnext/appshome/data/<APPID>/user.env`.

### Usefull environment Variables while running the compose

While running the initscript there are some PLCnext specific environment variables available which should be used for specific tasks. They are the following and offering the path to:

- the mountpoint for ro app file: `APP_PATH`
- the app temporary data storage: `APP_TMP_PATH`
- the app persistent data storage: `APP_DATA_PATH`
- the app specific logfile: `APP_LOG`

### Volumes and storage

First of all you have to use the app specific space for temporary and persistant data of your app. The available environment variables can be used for this task, e.g.:

```yml
    volumes:
      - $APP_DATA_PATH/data:/data
```

To make them accessible as admin, you have to add the same host pathes to the init script as well. Use and extend the space separated `VOLUMES` list like this:

```bash
VOLUMES=("$APP_DATA_PATH/data" "$APP_DATA_PATH/test2")
```
