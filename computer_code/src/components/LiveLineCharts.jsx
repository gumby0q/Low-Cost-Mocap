// LiveLineChart.js
import React, { useState, useEffect } from 'react';
import { createRoot } from 'react-dom/client';
import { Line } from 'react-chartjs-2';
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from 'chart.js';
// import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import { socket } from '../shared/styles/scripts/socket';

// ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);
ChartJS.register(CategoryScale, LinearScale, LineElement, PointElement, Title, Legend);

const MAX_DATA_LENGTH = 100


const serial_data_header = ["xPWM", "yPWM", "zPWM", "yawPWM",
  "xPos", "yPos", "zPos", "yawPos",
  "xVelSetpoint", "yVelSetpoint", "zVelSetpoint",
  "xVelOutput", "yVelOutput", "zVelOutput", "yawPosOutput",
  "groundEffectMultiplier", "millis","armed", "max_t_diff_us",
]


// const generateColors = (numColors) => {
//   const colors = [];
//   const step = 360 / numColors; // Equal distribution of hues around the color wheel

//   for (let i = 0; i < numColors; i++) {
//     const hue = i * step;
//     const color = `hsl(${hue}, 100%, 50%)`;
//     colors.push(color);
//   }

//   return colors;
// };

const generateColors = (numColors) => {
  const colors = [];
  const step = 360 / numColors; // Equal distribution of hues around the color wheel

  for (let i = 0; i < numColors; i++) {
    const hue = i * step;
    const solidColor = `hsl(${hue}, 100%, 50%)`;
    const transparentColor = `hsla(${hue}, 100%, 50%, 0.2)`;
    colors.push({ solidColor, transparentColor });
  }

  return colors;
};

const millisIndex = serial_data_header.indexOf('millis');
// const zPWMIndex = serial_data_header.indexOf('zPWM');

// const dataToShowNames = [
//   'max_t_diff_us',
//   'xPWM',
//   'yPWM',
//   'zPWM',
// ];
// const dataToShowIndexes = dataToShowNames.map(i => serial_data_header.indexOf(i))
// const dataLength = dataToShowIndexes.length;
// const colors = generateColors(dataLength);




const LiveLineChart = ({ dataToShowNames, dataToShowIndexes, colors }) => {
  const [chartData, setChartData] = useState({
    labels: [], // Initialize with empty labels
    datasets: dataToShowNames.map((item, index) => {
      return {
        label: item,
        data: [],
        // fill: false,
        backgroundColor: colors[index].transparentColor,
        borderColor: colors[index].solidColor,
      }
    })
  });


  useEffect(() => {
    const listener = (data) => {
      const __data = data["data"];
      const newTime = __data[millisIndex];
      
      setChartData((prevData) => {
        const updatedLabels = [...prevData.labels, newTime];
        
        const updatedDataList = dataToShowIndexes.map((orderIndex, index) => {
          const newData = __data[orderIndex]; // Replace with your data source
          const updatedData = [...prevData.datasets[index].data, newData];
          if (updatedLabels.length > MAX_DATA_LENGTH) {
            updatedData.shift();
          }
          return updatedData;
        })

        if (updatedLabels.length > MAX_DATA_LENGTH) {
          updatedLabels.shift();
        }

        const _datasets = updatedDataList.map((data, index) => {
          return {
              ...prevData.datasets[index],
              data: data,
            }
        })

        return {
          labels: updatedLabels,
          datasets: _datasets,
        };
      });
    }
    socket.on("serial-port-data", listener)

    return () => {
      socket.off("serial-port-data", listener)
    }
  }, [setChartData])

  return (
    <Line 
      // style={{ height: 100 }}
      data={chartData}
      options={{
        animation: false,
        elements: {
          point: {
            radius: 0, // This removes the dots
          }
        }
      }}
      width={700}
      height={200}
    />
  );
};

const dataToShowNames = [
  // 'max_t_diff_us',
  "xVelSetpoint",
  "yVelSetpoint",
  "zVelSetpoint",
  "yawPosOutput",
];
const dataToShowIndexes = dataToShowNames.map(i => serial_data_header.indexOf(i))
const dataLength = dataToShowIndexes.length;
const colors = generateColors(dataLength);

// const dataToShowNames2 = [
//   'xPWM',
//   'yPWM',
//   'zPWM',
// ];

const dataToShowNames2 = [
  "xVelOutput",
  "yVelOutput",
  "zVelOutput",
];
const dataToShowIndexes2 = dataToShowNames2.map(i => serial_data_header.indexOf(i))
const dataLength2 = dataToShowIndexes2.length;
const colors2 = generateColors(dataLength2);

const dataToShowNames3 = [
  "xPWM", "yPWM", "zPWM", "yawPWM"
];
const dataToShowIndexes3 = dataToShowNames3.map(i => serial_data_header.indexOf(i))
const dataLength3 = dataToShowIndexes3.length;
const colors3 = generateColors(dataLength3);

const LiveLineCharts = () => {
    
  return (
    <div>
      <div>
        <p>Live Line Chart</p>
        <LiveLineChart
          colors={colors}
          dataToShowIndexes={dataToShowIndexes}
          dataToShowNames={dataToShowNames}
        />
      </div>
      <div>
        <p>Live Line Chart 2</p>
        <LiveLineChart
          colors={colors2}
          dataToShowIndexes={dataToShowIndexes2}
          dataToShowNames={dataToShowNames2}
        />
      </div>
      <div>
        <p>Live Line Chart 3</p>
        <LiveLineChart
          colors={colors3}
          dataToShowIndexes={dataToShowIndexes3}
          dataToShowNames={dataToShowNames3}
        />
      </div>
    </div>
  );
};

export default LiveLineCharts;


// OpenNewWindow.jsx
// import React, { useState, useEffect } from 'react';
// import ReactDOM from 'react-dom';

function copyStyles(sourceDoc, targetDoc) {
  Array.from(sourceDoc.styleSheets).forEach(styleSheet => {
    if (styleSheet.cssRules) { // for <style> elements
      const newStyleEl = sourceDoc.createElement('style');

      Array.from(styleSheet.cssRules).forEach(cssRule => {
        // write the text of each rule into the body of the style element
        newStyleEl.appendChild(sourceDoc.createTextNode(cssRule.cssText));
      });

      targetDoc.head.appendChild(newStyleEl);
    } else if (styleSheet.href) { // for <link> elements loading CSS from a URL
      const newLinkEl = sourceDoc.createElement('link');

      newLinkEl.rel = 'stylesheet';
      newLinkEl.href = styleSheet.href;
      targetDoc.head.appendChild(newLinkEl);
    }
  });
}

const OpenNewWindow = ({ innerComponent, label }) => {
  // const [newWindow, setNewWindow] = useState(null);

  // useEffect(() => {
  //   if (newWindow) {
  //     const newWindowDocument = newWindow.document;
  //     newWindowDocument.write('<div id="new-window-root"></div>');
  //     // ReactDOM.render(innerComponent, newWindowDocument.getElementById('new-window-root'));
  //     // const root = createRoot(newWindowDocument, options?)
  //     const root = createRoot(newWindowDocument)
  //     root.render(innerComponent) 

  //     copyStyles(document, newWindowDocument)

  //     // Cleanup when the component is unmounted or window is closed
  //     return () => {
  //       // newWindow.close();
  //       root.unmount();
  //     };
  //   }
  // }, [newWindow]);

  const [newWindow, setNewWindow] = useState(null);

  useEffect(() => {
    if (newWindow) {
      const newWindowDocument = newWindow.document;
      const newWindowRoot = newWindowDocument.createElement('div');
      newWindowRoot.id = 'new-window-root';
      newWindowDocument.body.appendChild(newWindowRoot);

      const root = createRoot(newWindowRoot);
      root.render(innerComponent);

      copyStyles(document, newWindowDocument);

      // Cleanup when the component is unmounted or window is closed
      const interval = setInterval(() => {
        if (newWindow.closed) {
          clearInterval(interval);
          root.unmount();
        }
      }, 1000);

      return () => {
        clearInterval(interval);
        root.unmount();
        newWindow.close();
      };
    }
  // }, [newWindow, innerComponent]);
  }, [newWindow]);

  const openNewWindow = () => {
    const win = window.open('', '', 'width=800,height=600');
    setNewWindow(win);
  };

  return (
    <div>
      <button onClick={openNewWindow}>{label}</button>
    </div>
  );
};

export {
  OpenNewWindow
};

