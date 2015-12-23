package paquete;

import java.awt.TextField;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;

import javax.swing.JCheckBox;
import javax.swing.JTextField;

public class Controlador extends Thread{

	Modelo modelo;
	Proxy mensajero;
	CapturadorContinuo capturadorContinuo;

	class CapturadorContinuo extends Thread
	{

		@Override
		public void run() {
			// TODO Auto-generated method stub
			DatosRssi datos;

			 
			if(mensajero==null) mensajero=new Proxy(modelo.getSocket());
			while(!Thread.interrupted())
			{
				try {
					//System.out.println("prueba5");
					
					datos=mensajero.obtenerRssiBaliza1(modelo.getNumMuestras());
					//System.out.println("prueba6");
					if(modelo.getCapturaContinua())
						modelo.actualizarRssiBaliza0(datos);
					//System.out.println("prueba7");
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					System.out.println("El thread no ha podido obtener los datos");
					break;
				}
				
			}

			
		}
		
	}
	Controlador(Modelo m)
	{
		modelo=m;
		capturadorContinuo = new CapturadorContinuo();
	}

	public void addModelo(Modelo m)
	{
		modelo = m;
	}
	/*
	public void conectar() throws IOException
	{
		InetAddress address;
		address = InetAddress.getByName(modelo.getServerIp());
		mensajero = new Proxy(address,modelo.getServerPort());
		
	}
	*/
	class PcWifiUnoWindowListener extends WindowAdapter{


		@Override
		public void windowClosing(WindowEvent arg0) {
			// TODO Auto-generated method stub

			System.exit(0);
			try
			{
				mensajero.close();
			}
			catch(Exception e)
			{
				System.out.println("Error al cerrar la conexion");
				e.printStackTrace();
			}

		}

	}


	public class DistInputListener extends KeyAdapter{

		@Override
		//public void keyTyped(KeyEvent arg0) {
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			char c = arg0.getKeyChar();
			JTextField input = (JTextField)arg0.getSource();

			if(c==KeyEvent.VK_BACK_SPACE)
			{
				if(input.getText().length()==0)
					modelo.distanciaValida(-1);
				else
				{
					modelo.distanciaValida(Float.parseFloat(input.getText()));
					System.out.println(input.getText());
				}
			}
			else
			{
				String texto;
				texto = input.getText();

				if(texto.matches("([0-9]|[1-9][0-9]*)(\\.[0-9]*|)"))
				{
					System.out.println(texto);
					modelo.distanciaValida(Float.parseFloat(texto));
				}
				else
				{
					System.out.println("distancia no v�lida");
					input.setText(texto.substring(0, texto.length()-1));
				}
			}

		}
	}
	
	public class RangeInputListener extends KeyAdapter{

		@Override
		//public void keyTyped(KeyEvent arg0) {
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			char c = arg0.getKeyChar();
			MyTextField input = (MyTextField)arg0.getSource();
			
			if(c==KeyEvent.VK_BACK_SPACE)
			{
				if(!input.getText().matches("([-+]|)"))
			
					modelo.actualizarRangoEjes(input.getId(),Float.parseFloat(input.getText()));
					System.out.println(input.getText());
				
			}
			else
			{
				String texto;
				//texto = input.getText()+c;
				texto = input.getText();
				if(texto.matches("(([+-]|)([0-9]|[1-9][0-9]*))(\\.[0-9]*|)"))
				{
					System.out.println(texto);
					modelo.actualizarRangoEjes(input.getId(),Float.parseFloat(texto));
				}
				else
				{
					System.out.println("distancia no v�lida");
					if(!texto.matches("[+-]"))
						input.setText(texto.substring(0, texto.length()-1));
				}
			}

		}
	}
	
	public class NumMuestrasInputListener extends KeyAdapter{

		@Override
		//public void keyTyped(KeyEvent arg0) {
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			char c = arg0.getKeyChar();
			TextField input = (TextField)arg0.getSource();


			String texto;
			//texto = input.getText()+c;
			texto = input.getText();
			if(texto.matches("[1-9][0-9]*"))
			{
				System.out.println(texto);
				modelo.setNumMuestras(Integer.parseInt(texto));
			}
			else
			{
				System.out.println("distancia no v�lida");
				if(input.getText().length()>0)
					input.setText(texto.substring(0, texto.length()-1));
			}


		}
	}

	class Rssi0ButtonListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
			modelo.botonRssiPulsado();
			if(mensajero==null) mensajero=new Proxy(modelo.getSocket());
			
			try {
				DatosRssi datos=mensajero.obtenerRssiBaliza1(modelo.getNumMuestras());
				modelo.actualizarRssiBaliza0(datos);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			

		}

	}


	class Rssi1ButtonListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
			if(mensajero==null) mensajero=new Proxy(modelo.getSocket());
			modelo.botonRssiPulsado();
			try {
				DatosRssi datos=mensajero.obtenerRssiBaliza1(modelo.getNumMuestras());
				modelo.actualizarRssiBaliza1(datos);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}



		}
	}
	class LimpiarGraficaButtonListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
				modelo.limpiarGrafica();
		}
	}
	

	class CaptContCheckBoxListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
			JCheckBox check = (JCheckBox)arg0.getSource();
			modelo.setCapturaContinua(check.isSelected());
			if(check.isSelected())
			{
				capturadorContinuo=new CapturadorContinuo();
				capturadorContinuo.start();
				System.out.println("Thread lanzado");
			}
			else
			{
				if(capturadorContinuo!=null && capturadorContinuo.isAlive())
					capturadorContinuo.interrupt();
				
			}
			

		}

	}
	class EjeYfijoCheckBoxListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
			JCheckBox check = (JCheckBox)arg0.getSource();
			modelo.setEjeYfijo(check.isSelected());
			

		}

	}
	
	class EjeXfijoCheckBoxListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			// TODO Auto-generated method stub
			JCheckBox check = (JCheckBox)arg0.getSource();
			modelo.setEjeXfijo(check.isSelected());
			

		}

	}
	
	public ActionListener getEjeYfijoActionListener()
	{
		return new EjeYfijoCheckBoxListener();
	}
	
	public ActionListener getEjeXfijoActionListener()
	{
		return new EjeXfijoCheckBoxListener();
	}
	
	public ActionListener getCaptContListener()
	{
		return new CaptContCheckBoxListener();
	}
	
	public KeyAdapter getNumMuestrasInputListener()
	{
		return new NumMuestrasInputListener();
	}
	public KeyAdapter getRangeInputListener()
	{
		return new RangeInputListener();
	}
	public WindowAdapter getPcWifiUnoWindowListener()
	{
		return new PcWifiUnoWindowListener();
	}

	public Rssi0ButtonListener getRssi0ButtonListener()
	{
		return new Rssi0ButtonListener();
	}


	public Rssi1ButtonListener getRssi1ButtonListener()
	{
		return new Rssi1ButtonListener();
	}
	
	public KeyAdapter getDistInputListener()
	{
		return new DistInputListener();
	}
	
	public ActionListener getLimpiarGraficaButtonListener()
	{
		return new LimpiarGraficaButtonListener();
	}
	

}
	



