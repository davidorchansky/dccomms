package paquete;
import javax.swing.*;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;

import java.net.SocketException;
import java.net.UnknownHostException;


public class VistaPeticionDatosController extends Thread {

	VistaPeticionDatos vista;
	Modelo model;
	
	
	VistaPeticionDatosController(Modelo m)
	{
		model=m;
	}
	public void addModel(Modelo m)
	{
		model = m;
	}
	
	public void addVistaPeticionDatos(VistaPeticionDatos v){
		vista = v;
		
	}
	
	public class OkButtonListener implements ActionListener{

		@Override
		public void actionPerformed(ActionEvent arg0) {
			//Window ventana = SwingUtilities.getWindowAncestor((Component) arg0.getSource());
			//ventana.dispose();
			peticionSalida();
			
			
		}
		
	}
	

	
	public class IPInputListener extends KeyAdapter{

		@Override
		public void keyTyped(KeyEvent arg0) {
			// TODO Auto-generated method stub
			char c = arg0.getKeyChar();
			if (c == '\n')
				peticionSalida();
			else if (c==' ')
				arg0.consume();
			
			//System.out.println(arg0.getKeyChar());

			
			
		}
		@Override
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			JTextField input = (JTextField)arg0.getSource();
			String texto = input.getText();
			model.cambiarIPserver(texto);
		}
		
		
	}
	
	public class PortInputListener extends KeyAdapter{

		@Override
		public void keyTyped(KeyEvent arg0) {
			// TODO Auto-generated method stub
			char c = arg0.getKeyChar();
			if (c == '\n')
				peticionSalida();
			
			//System.out.println(arg0.getKeyChar());
			else if (c<'0' || c>'9')
				arg0.consume();
			
			
		}
		@Override
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			JTextField input = (JTextField)arg0.getSource();
			String texto = input.getText();
			model.cambiarPortServer(texto);
			
		}
		
		
	}
	private void peticionSalida()
	{
		
		if(model.peticionSalida())
		
			if(!conectarConServidor())
			{
				model.conexionFallida();
				System.exit(1);

			}
		
		
	}
	
	public boolean conectarConServidor()
	{
		InetAddress address;
		try {
			address = InetAddress.getByName(model.getServerIp());
			model.setSocket(new Socket(address,model.getServerPort()));
			return true;
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return false;
		
	}
	public OkButtonListener getOkButtonListener()
	{
		return new OkButtonListener();
	}
	

	
	public IPInputListener getIPInputListener()
	{
		return new IPInputListener();
	}
	
	public PortInputListener getPortInputListener()
	{
		return new PortInputListener();
	}
	
}
