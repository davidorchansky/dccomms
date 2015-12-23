package paquete;

import java.net.Socket;


public class Modelo implements IrssiBaliza0ButtonSubject, IrssiBaliza1ButtonSubject, IrxSubject, IInputSubject, IGraficaSubject {

	private IrxObserver rx;
	private IrssiBaliza0ButtonObserver rssiBaliza0Button;
	private IrssiBaliza1ButtonObserver rssiBaliza1Button;

	private DatosRssi rssiBaliza0;
	private DatosRssi rssiBaliza1;
	private IGraficaObserver graficaObserver;
	
	private int numMuestras=1;
	
	private int serverPort = 8001;
	//private String serverIP = "169.254.1.1"; //IP Wifly modo Ad-hoc
	private String serverIP = "localhost";
	private boolean puertoValido = true;
	
	private boolean ejeXfijo=false;
	private boolean ejeYfijo=false;
	private float ejeXlower = 0;
	private float ejeXupper = 0;
	private float ejeYlower = 0;
	private float ejeYupper = 0;
	
	private boolean capturaContinua = false;
	private float distancia = 0;
	
	
	private Socket socket;
	
	private IInputObserver botonOk;
	
	
	public void setCapturaContinua(boolean b)
	{
		capturaContinua=b;
		graficaObserver.cambiarModo();
	}
	public void actualizarRangoEjes(int id, float r)
	{
		if(id==0)
		{
			ejeXlower=r;
			graficaObserver.updateEjeX();
		}
		else if(id==1)
		{
			ejeXupper=r;
			graficaObserver.updateEjeX();
		}
		else if(id==2)
		{
			ejeYlower=r;
			graficaObserver.updateEjeY();
		}
		else if(id==3)
		{
			ejeYupper=r;
			graficaObserver.updateEjeY();
		}
	}
	public void setEjeYfijo(boolean b)
	{
		ejeYfijo=b;
		graficaObserver.updateEjeY();
		
	}
	
	public void setEjeXfijo(boolean b)
	{
		ejeXfijo=b;
		graficaObserver.updateEjeX();	
	}
	public boolean getEjeYfijo()
	{
		return ejeYfijo;
	}
	public boolean getEjeXfijo()
	{
		return ejeXfijo;
	}
	public float getEjeYupper()
	{
		return ejeYupper;
	}
	public float getDistancia()
	{
		return distancia;
	}
	public float getEjeXupper()
	{
		return ejeXupper;
	}
	
	public float getEjeXlower()
	{
		return ejeXlower;
	}
	public float getEjeYlower()
	{
		return ejeYlower;
	}
	
	public boolean getCapturaContinua()
	{
		return capturaContinua;
	}
	public void actualizarRssiBaliza0(DatosRssi d)
	{
		rssiBaliza0 = d;
		/*String s="*** BALIZA 1 ***\n"
				+ "RSSI ida:\t"+d.getRssiIda()+"\n"
				+ "RSSI vuelta:\t"+d.getRssiVuelta()+"\n";*/
		String s="RSSI: "+d.getRssiIda();
		notifyRxObservers(s);
		graficaObserver.addMuestra(distancia, d.getRssiIda(), d.getRssiVuelta());
		
	}
	
	public void actualizarRssiBaliza1(DatosRssi d)
	{
		rssiBaliza1 = d;
		/*String s="*** BALIZA 2 ***\n"
				+ "RSSI ida:\t"+d.getRssiIda()+"\n"
				+ "RSSI vuelta:\t"+d.getRssiVuelta()+"\n";*/
		String s="RSSI: "+d.getRssiIda();
		notifyRxObservers(s);
		graficaObserver.addMuestra(distancia, d.getRssiIda(), d.getRssiVuelta());
	}

	
	public void cambiarPortServer(String port)
	{
		if (port.length()>0)
		{
			setServerPort(Integer.parseInt(port));
			puertoValido = true;
			if (serverIP.length()>0)
				notifyInputObservers(1);
			else
				notifyInputObservers(0);
		}
		else
		{
			puertoValido = false;
			notifyInputObservers(0);
		}
	}
	
	public void cambiarIPserver(String ip)
	{
		serverIP = ip;
		if (ip.length()>0 && puertoValido)
				notifyInputObservers(1);
	
		else
			notifyInputObservers(0);
	}
	
	public boolean peticionSalida(){
		if (puertoValido && serverIP.length()>0)
		{
			notifyInputObservers(2);
			return true;
		}
		return false;
	}
	
	public void conexionFallida()
	{
		notifyInputObservers(3);
	}
	
	@Override
	public void registerRxObserver(IrxObserver ob) {
		// TODO Auto-generated method stub
		rx=ob;
	}
	@Override
	public void removeRxObserver(IrxObserver ob) {
		// TODO Auto-generated method stub
		rx=null;
	}
	@Override
	public void notifyRxObservers(String s) {
		// TODO Auto-generated method stub
		rx.updateRxObserver(s);
	}
	@Override
	public void registerRssiBaliza1ButtonObserver(
			IrssiBaliza1ButtonObserver observer) {
		// TODO Auto-generated method stub
		rssiBaliza1Button=observer;
	}
	@Override
	public void removeRssiBaliza1ButtonObserver(IrssiBaliza1ButtonObserver o) {
		// TODO Auto-generated method stub
		rssiBaliza1Button=null;
	}
	@Override
	public void notifyRssiBaliza1ButtonObservers(boolean activo) {
		// TODO Auto-generated method stub
		rssiBaliza1Button.updateRssiBaliza1Button(activo);
		
	}
	@Override
	public void registerRssiBaliza0ButtonObserver(IrssiBaliza0ButtonObserver observer) {
		// TODO Auto-generated method stub
		rssiBaliza0Button=observer;
	}
	@Override
	public void removeRssiBaliza0ButtonObserver(IrssiBaliza0ButtonObserver o) {
		// TODO Auto-generated method stub
		rssiBaliza0Button=null;
	}
	@Override
	public void notifyRssiBaliza0ButtonObservers(boolean activo) {
		// TODO Auto-generated method stub
		rssiBaliza0Button.updateRssiBaliza0Button(activo);
	}

	public int getServerPort() {
		return serverPort;
	}

	public void setServerPort(int serverPort) {
		this.serverPort = serverPort;
	}
	
	public String getServerIp() {
		return serverIP;
	}

	public void setServerIP(String add) {
		serverIP = add;
	}

	@Override
	public void registerInputObserver(IInputObserver o) {
		// TODO Auto-generated method stub
		botonOk = o;
	}

	@Override
	public void removeInputObserver(IInputObserver o) {
		// TODO Auto-generated method stub
		botonOk = null;
	}

	@Override
	public void notifyInputObservers(int evento) {
		// TODO Auto-generated method stub
		botonOk.updateView(evento);
	}

	public Socket getSocket() {
		return socket;
	}

	public void setSocket(Socket socket) {
		this.socket = socket;
	}
	
	public void botonRssiPulsado()
	{
		
		//notifyRxObservers("Obteniendo datos...");
	}

	public void distanciaValida(float dist)
	{
		if(dist>=0)
		{
			distancia=dist;
			notifyRssiBaliza0ButtonObservers(true);
			notifyRssiBaliza1ButtonObservers(true);
		}
		else
		{
			notifyRssiBaliza0ButtonObservers(false);
			notifyRssiBaliza1ButtonObservers(false);
		}
	}
	@Override
	public void registerGraficaObserver(IGraficaObserver observer) {
		// TODO Auto-generated method stub
		graficaObserver=observer;
		graficaObserver.updateEjeX();
		graficaObserver.updateEjeY();
		graficaObserver.cambiarModo();
		graficaObserver.inicializarVista();
		
	}

	public void limpiarGrafica()
	{
		graficaObserver.limpiarGrafica();
	}
	public int getNumMuestras() {
		return numMuestras;
	}
	public void setNumMuestras(int numMuestras) {
		this.numMuestras = numMuestras;
	}
	
	
}
