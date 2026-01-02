import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ChatWidget from '@site/src/components/ChatWidget';

// Wraps the entire app to inject global components
export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => <ChatWidget />}
      </BrowserOnly>
    </>
  );
}
