package paquete;

import javax.swing.JTextField;

public class MyTextField extends JTextField{

	private int id;
	public MyTextField(int identificador,int cols)
	{
		super(cols);
		id = identificador;
	}
	
	public int getId()
	{
		return id;
	}
	
}
