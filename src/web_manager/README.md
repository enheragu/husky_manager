# Husky Web Manager

Interfaz web para monitorizar y gestionar el sistema Husky ROS.

## Configuración de Sudoers (requerido para relanzar servicios)

Para poder reiniciar servicios desde la interfaz web sin que pida contraseña:

```bash
# Copiar el archivo de configuración de sudoers
sudo cp src/web_manager/sudoers.d/husky-manager /etc/sudoers.d/husky-manager

# Establecer permisos correctos (IMPORTANTE)
sudo chmod 440 /etc/sudoers.d/husky-manager

# Verificar que la sintaxis es correcta
sudo visudo -c
```

> **Nota**: Si el usuario que ejecuta el web manager no es `administrator`, edita el archivo `/etc/sudoers.d/husky-manager` y cambia `administrator` por el usuario correcto.

## Uso

El web manager se inicia automáticamente con el servicio `husky_web_manager.service` o manualmente:

```bash
rosrun husky_manager web_manager_node.py
```

Accede a la interfaz en: `http://<IP_HUSKY>:5050/manager`
