HOW TO INSTALL:

1. Copiar "04_rfcockpit" a "/opt/openrov/dashboard/src/plugins"
2. Copiar "rf-src" a "/opt/openrov/dashboard"
3. Instalar los scripts de inicio:

Instalamos los scripts de inicio contenidos en el directorio "scripts", por ejemplo, ustilizado la
herramienta "update-rc.d" (aconsejable tener estos scripts en "/opt/openrov/linux" y usar enlaces simbólicos
desde los subdirectorios de "/etc/*"):

(enlace de interés: http://www.alvarolara.com/2013/03/20/ejecutar-un-script-al-iniciar-sesion-en-ubuntu/)

"radioSetup.sh" configura el un puerto serie para comunicarse con el transmisor cuando no se usa una MCU,
es decir, cuando el transmisor es un módulo que modula directamente la señal que recibe por uno de sus
pines.

"rfopenrov.sh" es un script que se usa para lanzar o parar el plugin "video_through_rf" desde el dashboard

