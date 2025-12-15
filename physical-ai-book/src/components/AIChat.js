import React, { useState, useRef, useEffect } from 'react';

const AIChat = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState([]); // Stores { text, sender: 'user' | 'ai' }
  const [loading, setLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage = { text: input, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const res = await fetch('https://my-rag-chatbot-ipx8.onrender.com/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ message: input }),
      });

      if (!res.ok) {
        throw new Error(`HTTP error! status: ${res.status}`);
      }

      const data = await res.json();
      const aiMessage = { text: data.response, sender: 'ai' };
      setMessages((prevMessages) => [...prevMessages, aiMessage]);
    } catch (error) {
      console.error("Failed to fetch:", error);
      const errorMessage = { text: 'Sorry, something went wrong.', sender: 'ai' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const styles = {
    chatButton: {
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      backgroundColor: '#007bff',
      color: 'white',
      border: 'none',
      borderRadius: '50%',
      width: '60px',
      height: '60px',
      fontSize: '24px',
      cursor: 'pointer',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
      zIndex: 1000,
    },
    chatWindow: {
      position: 'fixed',
      bottom: '90px',
      right: '20px',
      width: '350px',
      height: '500px',
      backgroundColor: 'white',
      borderRadius: '10px',
      boxShadow: '0 5px 15px rgba(0,0,0,0.3)',
      display: 'flex',
      flexDirection: 'column',
      zIndex: 1000,
      overflow: 'hidden',
    },
    chatHeader: {
      backgroundColor: '#007bff',
      color: 'white',
      padding: '10px',
      borderTopLeftRadius: '10px',
      borderTopRightRadius: '10px',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
    },
    closeButton: {
      backgroundColor: 'transparent',
      border: 'none',
      color: 'white',
      fontSize: '20px',
      cursor: 'pointer',
    },
    messagesContainer: {
      flexGrow: 1,
      padding: '10px',
      overflowY: 'auto',
      display: 'flex',
      flexDirection: 'column',
      backgroundColor: '#f5f5f5',
    },
    messageBubble: {
      maxWidth: '80%',
      padding: '8px 12px',
      borderRadius: '15px',
      marginBottom: '8px',
      wordWrap: 'break-word',
    },
    userMessage: {
      alignSelf: 'flex-end',
      backgroundColor: '#28a745', // Green
      color: 'white',
      borderBottomRightRadius: '2px',
    },
    aiMessage: {
      alignSelf: 'flex-start',
      backgroundColor: '#e0e0e0', // Gray
      color: 'black',
      borderBottomLeftRadius: '2px',
    },
    inputForm: {
      display: 'flex',
      padding: '10px',
      borderTop: '1px solid #eee',
    },
    inputField: {
      flexGrow: 1,
      padding: '10px',
      borderRadius: '20px',
      border: '1px solid #ddd',
      marginRight: '10px',
    },
    sendButton: {
      padding: '8px 15px',
      border: 'none',
      borderRadius: '20px',
      backgroundColor: '#007bff',
      color: 'white',
      cursor: 'pointer',
    },
    loadingMessage: {
      textAlign: 'center',
      fontStyle: 'italic',
      color: '#555',
      padding: '10px',
    }
  };

  return (
    <>
      {!isChatOpen && (
        <button style={styles.chatButton} onClick={() => setIsChatOpen(true)}>
          💬
        </button>
      )}

      {isChatOpen && (
        <div style={styles.chatWindow}>
          <div style={styles.chatHeader}>
            <span>AI Chat</span>
            <button style={styles.closeButton} onClick={() => setIsChatOpen(false)}>
              ✕
            </button>
          </div>
          <div style={styles.messagesContainer}>
            {messages.map((msg, index) => (
              <div
                key={index}
                style={{
                  ...styles.messageBubble,
                  ...(msg.sender === 'user' ? styles.userMessage : styles.aiMessage),
                }}
              >
                {msg.text}
              </div>
            ))}
            {loading && (
                <div style={{...styles.messageBubble, ...styles.aiMessage, ...styles.loadingMessage}}>
                    Thinking...
                </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} style={styles.inputForm}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Type your message..."
              style={styles.inputField}
              disabled={loading}
            />
            <button type="submit" disabled={loading} style={styles.sendButton}>
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default AIChat;