import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class All0000FFFF {
  public static void main(String[] args) {
    try {
      FileWriter f = new FileWriter("hex0000toFFFF.txt");
      for (int i = 0; i < 0x10000; i++) {
        f.write(String.format("%04X", i) + '\n');
      }
      f.close();
      System.out.println("Successfully wrote to the file.");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }
}
