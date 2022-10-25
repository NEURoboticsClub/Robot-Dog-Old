package com.example.http_example;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.io.IOException;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;

public class MainActivity extends AppCompatActivity {
    private TextView mTextViewResult;

    int n; //for the user inputted number
    EditText oddEvenInput; //for the user input before converted to integer
    Button submitButton; //button object ... same name as xml file


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        mTextViewResult = findViewById(R.id.text_view_result);
        OkHttpClient client = new OkHttpClient();

        oddEvenInput=(EditText)findViewById(R.id.oddEvenInput); //connect to text input in xml file
        submitButton= (Button) findViewById(R.id.submitButton);//connect to button in xml file

        submitButton.setOnClickListener(new View.OnClickListener() { //onclick listener for submit button
            @Override
            public void onClick(View view) {
                n=Integer.valueOf(oddEvenInput.getText().toString()); //convert the input to an integer
                String url = "http://69.164.212.94:1500/isEven?n="+n;
                makeRequest(url, client);

            }
        });

        String url = "http://69.164.212.94:1500/isEven?n=5";
        makeRequest(url, client);

    }
    public void makeRequest(String url,  OkHttpClient client){
        Request request = new Request.Builder()
                .url(url)
                .build();
        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(@NonNull Call call, @NonNull IOException e) {

                MainActivity.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mTextViewResult.setText(e.toString());

                    }
                });
            }

            @Override
            public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                if (response.isSuccessful()){
                    String myResponse = response.body().string();

                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mTextViewResult.setText(myResponse);

                        }
                    });
                }
            }
        });
    }


}