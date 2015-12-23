package paquete;

public class PcWifiUno{

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Modelo modelo = new Modelo();
		
		VistaPeticionDatosController vpdC= new VistaPeticionDatosController(modelo);
		VistaPeticionDatos vpd=new VistaPeticionDatos(vpdC,modelo);
		
		vpd.show();
		
		Vista vistaPrincipal = new Vista(modelo);
		modelo.registerRssiBaliza0ButtonObserver(vistaPrincipal);
		modelo.registerRssiBaliza1ButtonObserver(vistaPrincipal);
		modelo.registerRxObserver(vistaPrincipal);
		modelo.registerGraficaObserver(vistaPrincipal);
		vistaPrincipal.addController(new Controlador(modelo));

		vistaPrincipal.show();
		
	}

}
