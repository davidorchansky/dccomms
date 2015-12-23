package paquete;
import javax.swing.*;

import javax.swing.JDialog;



public class VistaPeticionDatos implements IInputObserver{
	
	
	private JDialog dialogo;

	private JTextField serverIp;
	private JTextField serverPort;
	private JButton okbutton;
	
	

	public VistaPeticionDatos(VistaPeticionDatosController controller,IInputSubject myModel){

		serverIp = new JTextField(20);
		//serverIp.setText("169.254.1.1"); //IP Wifly modo Ad-hoc
		serverIp.setText("localhost");	   
		serverPort = new JTextField(20);
		serverPort.setText("8001");
		okbutton = new JButton("Ok");
		dialogo = new JDialog(new JFrame(), "Datos de conexion",true );
		addController(controller);
		myModel.registerInputObserver(this);
		controller.addVistaPeticionDatos(this);
		
		
	}
	
	
	public void addController(VistaPeticionDatosController controller)
	{
		
		okbutton.addActionListener(controller.getOkButtonListener());
		serverIp.addKeyListener(controller.getIPInputListener());
		serverPort.addKeyListener(controller.getPortInputListener());
		
	}

	
	public void show()
	{

		int tamy=200, tamx=200;
		dialogo.setSize(tamy,tamx);
		dialogo.setLocation(-tamx/2,-tamy/2);
		dialogo.setLocationRelativeTo(null);
		dialogo.setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
		
		
		JLabel label2 = new JLabel("IP (o nombre) del servidor : ");
		JLabel label3 = new JLabel("Puerto del servidor: ");
		
	
		JPanel inputpanel = new JPanel();
		inputpanel.setLayout(new BoxLayout(inputpanel,BoxLayout.Y_AXIS));
		

		JPanel ip = new JPanel();
		JPanel puerto = new JPanel();
		


		
		
		ip.add(label2);
		ip.add(serverIp);
		
		puerto.add(label3);
		puerto.add(serverPort);
		

		inputpanel.add(ip);
		inputpanel.add(puerto);
		inputpanel.add(okbutton);
		
		//okbutton.setEnabled(false);
	
		
	
		dialogo.getContentPane().add(inputpanel);
		dialogo.setSize(500,150);
		dialogo.setVisible(true);
		
		

	}

	@Override
	public void updateView(int evento) {
		// TODO Auto-generated method stub
		if (evento == 0)
			okbutton.setEnabled(false);
		else if (evento == 1)
			okbutton.setEnabled(true);
		else if (evento == 2)
				dialogo.dispose();
		else
		{
			JOptionPane.showMessageDialog(new JFrame(),
				    "No se ha podido establecer conexión",
				    "Error de conexión",
				    JOptionPane.ERROR_MESSAGE);
		}
	}
}
		