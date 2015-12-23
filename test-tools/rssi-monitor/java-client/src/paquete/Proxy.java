package paquete;


import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketAddress;

public class Proxy {

	private Socket socket;
	//private PrintWriter salida;
	private BufferedReader entrada;
	private InputStream input;
	//private InputStream salida;
	//private OutputStream entrada;
	private DataOutputStream salida;
	//private DataInputStream entrada;
	//***private InputStreamReader entrada;

	private byte DIRBALIZA0=0x01;
	private byte DIRBALIZA1=0x02;
	private byte DIRBALIZA2=0x03;
	private byte DIRBALIZA3=0x04;

	
	Proxy(InetAddress serverAdd, int portDest) throws IOException
	{
			socket=new Socket(serverAdd,portDest);
			input=socket.getInputStream();
			//salida=new PrintWriter(socket.getOutputStream(),true);
			entrada=new BufferedReader(new InputStreamReader(socket.getInputStream()));
			//salida=socket.getOutputStream();
			//entrada=socket.getInputStream();
			//***entrada=new InputStreamReader(socket.getInputStream());
			salida=new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));
			//entrada=new DataInputStream(new BufferedInputStream(socket.getInputStream()));

	}
	
	Proxy(Socket s)
	{
			socket=s;
			
			try {
				input=socket.getInputStream();
				salida=new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));
				//entrada=new DataInputStream(new BufferedInputStream(socket.getInputStream()));
				//salida=new PrintWriter(socket.getOutputStream(),true);
				entrada=new BufferedReader(new InputStreamReader(socket.getInputStream()));
				//***entrada=new InputStreamReader(socket.getInputStream());
				//salida=socket.getOutputStream();
				//entrada=socket.getInputStream();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.exit(1);
			}
			
			

	}
	
	public void close() throws IOException
	{
		socket.close();
	}
	
	public DatosRssi obtenerRssiBaliza1(int nmuestras) throws IOException
	{
		
		//entrada.reset();
		float a=0;
		float b=0;
		
		if(input.available()>0)
			input.skip(input.available());
		int i=0;
		while(i<nmuestras)
		{
			//socket.getInputStream().
			try
			{
				String respuesta = entrada.readLine();
				if (!respuesta.isEmpty())
				{
					String [] datos=respuesta.split(" ");
			
					a+=Float.parseFloat(datos[0]);
					
					b+=Float.parseFloat(datos[1]);
					System.out.println(i);
					i++;
				}
				else
					System.out.println("Error. Leido: "+ respuesta);
			}
			catch(Exception e)
			{
				System.out.println("Error al recibir datos");
				e.printStackTrace();
			}
		}
		a/=nmuestras;
		b/=nmuestras;
		DatosRssi datosRssi=new DatosRssi(a,b);
	
		return datosRssi;

	}
	public DatosRssi obtenerRssiBaliza2() throws IOException
	{
		
		//entrada.reset();
		String respuesta = entrada.readLine();	

		String [] datos=respuesta.split(" ");
		
		
		float a=Float.parseFloat(datos[0]);
		float b=Float.parseFloat(datos[1]);
	
		DatosRssi datosRssi=new DatosRssi(a,b);

		
		return datosRssi;
		
	}
	
}
